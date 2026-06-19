/// @file drone/sat/util/concurrency/lock.h
#pragma once
#include "drone/sat/util/perf_util/perf_util.h"
#include <atomic>
#include <cstdio>
namespace Drone::Sat
{
    /// Efficient realtime applications should never sleep or yield to the OS
    /// scheduler, but rather coordinate work by executing jobs on userspace
    /// fibers. Although some level of coordination is often required, mutual
    /// exclusion should be minimized via explicit depedency ordering or
    /// bucketing.
    ///
    /// If these options are exhausted and it's still necessary to enforce
    /// mutual exclusion, this class may be used. It provides a basic spinlock
    /// concurrency primitive. It is not intended for high-contention or
    /// long-running contention situations, but it does provide basic fairness
    /// management to mitigate starvation. It does NOT guarantee perfect
    /// fairness, provide priority inversion mitigation, or indeed have any
    /// (client-facing) notion of resource priority at all. A much more
    /// sophisticated implementation is required for this, but it is also
    /// typically unnecessary (see the aforementioned alternatives to highly
    /// contentious mutual exclusion).
    ///
    /// Consider instantiating as an alignas(128) struct member if you can, to
    /// avoid false sharing.
    class WARN_UNUSED Lock
    {
        /// Bits  0- 9: readers
        /// Bits 10-10: writer
        /// Bits 11-11: writer spinning
        /// Bits 12-63: priority
        /// @{
        static constexpr U64 kRead = 1ULL << 0;
        static constexpr U64 kWrite = 1ULL << 10;
        static constexpr U64 kSpin = 1ULL << 11;
        static constexpr U64 kPrio = 1ULL << 12;
        /// @}
    public:
        /// Constructor.
        ALWAYS_INLINE Lock() : m_x(0) {}
        /// Destructor.
        ALWAYS_INLINE ~Lock()
        {
            const U64 x = m_x.load(std::memory_order_relaxed);
            if (UNLIKELY(x))
            {
                fprintf(stderr,
                        "FATAL: lock destructed while still %s. Terminating.\n",
                        x & kWrite         ? "locked for write"
                        : x & (kWrite - 1) ? "locked for read"
                                           : "spinning");
                fflush(stdout);
                fflush(stderr);
                // Die even in production. There is NO safe way to continue in
                // this situation. We should probably have an ALWAYS_ASSERT(),
                // but this'll do just fine. Farewell, cruel world!
                // NOLINTNEXTLINE(hicpp-use-nullptr,modernize-use-nullptr,clang-analyzer-core.NullDereference)
                *(volatile int *)0 = 0;
            }
        }
        /// Take lock for write.
        ///
        /// Only one writer (and no readers) may hold the lock at any time,
        /// whereas multiple readers (and no writers) may hold the lock at once.
        ///
        /// There's usually no need to call this function directly; use the
        /// handy janitorial wrapper WriteLockJanitor to take care of cleanup
        /// for you, as it's easy to forget to cleanup (unlock) along all
        /// possible control flow paths.
        ALWAYS_INLINE void WriteLock()
        {
            for (U64 prio = kPrio;;)
            {
                U64 x = m_x.load(std::memory_order_relaxed);
                const U64 rw = x & (kSpin - 1);
                if (LIKELY(!rw) && LIKELY(prio > x))
                {
                    if (LIKELY(m_x.compare_exchange_weak(
                            x, kWrite, std::memory_order_acquire,
                            std::memory_order_relaxed)))
                        return;
                }
                else
                {
                    if (LIKELY(prio > x + 16 * kPrio))
                        m_x.compare_exchange_weak(x,
                                                  kSpin | (prio - kPrio) | rw,
                                                  std::memory_order_relaxed,
                                                  std::memory_order_relaxed);
                    prio += kPrio;
                }
                SPIN_WAIT_HINT();
            }
        }
        /// Take lock for read.
        ///
        /// Only one writer (and no readers) may hold the lock at any time,
        /// whereas multiple readers (and no writers) may hold the lock at once.
        ///
        /// There's usually no need to call this function directly; use the
        /// handy janitorial wrapper ReadLockJanitor to take care of cleanup for
        /// you, as it's easy to forget to cleanup (unlock) along all possible
        /// control flow paths.
        ALWAYS_INLINE void ReadLock()
        {
            for (U64 prio = kPrio;;)
            {
                U64 x = m_x.load(std::memory_order_relaxed);
                const U64 rw = x & (kSpin - 1);
                const U64 spinningWriter = x & kSpin;
                const bool writerPrecedence = rw && spinningWriter;
                if (LIKELY((rw < kWrite - 1) & !writerPrecedence & (prio > x)))
                {
                    if (LIKELY(m_x.compare_exchange_weak(
                            x, spinningWriter | (rw + kRead),
                            std::memory_order_acquire,
                            std::memory_order_relaxed)))
                        return;
                }
                else
                {
                    if (LIKELY(!writerPrecedence & (prio > x + 16 * kPrio)))
                        m_x.compare_exchange_weak(
                            x, spinningWriter | (prio - kPrio) | rw,
                            std::memory_order_relaxed,
                            std::memory_order_relaxed);
                    prio += kPrio;
                }
                SPIN_WAIT_HINT();
            }
        }
        /// Unlock a lock previously locked for write.
        ALWAYS_INLINE void WriteUnlock()
        {
            m_x.fetch_sub(kWrite, std::memory_order_release);
        }
        /// Unlock a lock previously locked for read.
        ALWAYS_INLINE void ReadUnlock()
        {
            m_x.fetch_sub(kRead, std::memory_order_release);
        }
        /// Try to take lock for write, but don't spin if unable to lock
        /// immediately.
        ///
        /// Returns:
        ///  - True on successful lock; you now hold the lock and must
        ///  WriteUnlock() it later.
        ///  - False on failure to lock; the lock state has not changed, and you
        ///  need not, and must not,
        ///    WriteUnlock() it later.
        ///
        /// This is rarely what you're looking for, but it can be useful for
        /// "lock asserts": situations where you need mutual exclusion but DO
        /// NOT want to spin, and have designed your system so you should never
        /// have to - that is, mutual exclusion is supposed to be guaranteed by
        /// other means. In such cases, it can be useful to try taking the lock
        /// without spinning, because you don't want to and don't expect to have
        /// to spin, and on failure you die, because that indicates a logic
        /// error in whatever mechanism was supposed to ensure mutual exclusion
        /// by other means.
        ///
        //////////////////////////////////
        /// WARNING: WARNING: WARNING: ///
        //////////////////////////////////
        ///
        /// DO NOT MANUALLY SPIN ON THIS FUNCTION.
        /// while (!lock.TryWriteLock()) {} // NO!!!
        ///
        /// This kind of naive spin sidesteps the fairness management provided
        /// by WriteLock(). In contentious environments this WILL result in
        /// starvation and you WILL die in a fire. DO NOT DO THIS. Use
        /// WriteLock() instead.
        [[nodiscard]] ALWAYS_INLINE bool TryWriteLock()
        {
            U64 x = m_x.load(std::memory_order_relaxed);
            if (UNLIKELY(x & (kSpin - 1)))
                return false;
            return m_x.compare_exchange_strong(x, kWrite,
                                               std::memory_order_acquire,
                                               std::memory_order_relaxed);
        }
        /// Try to take lock for read, but don't spin if unable to lock
        /// immediately.
        ///
        /// Returns:
        ///  - True on successful lock; you now hold the lock and must
        ///  ReadUnlock() it later.
        ///  - False on failure to lock; the lock state has not changed, and you
        ///  need not, and must not,
        ///    ReadUnlock() it later.
        ///
        /// This is rarely what you're looking for, but it can be useful for
        /// "lock asserts": situations where you need mutual exclusion but DO
        /// NOT want to spin, and have designed your system so you should never
        /// have to - that is, mutual exclusion is supposed to be guaranteed by
        /// other means. In such cases, it can be useful to try taking the lock
        /// without spinning, because you don't want to and don't expect to have
        /// to spin, and on failure you die, because that indicates a logic
        /// error in whatever mechanism was supposed to ensure mutual exclusion
        /// by other means.
        ///
        //////////////////////////////////
        /// WARNING: WARNING: WARNING: ///
        //////////////////////////////////
        ///
        /// DO NOT MANUALLY SPIN ON THIS FUNCTION.
        /// while (!lock.TryReadLock()) {} // NO!!!
        ///
        /// This kind of naive spin sidesteps the fairness management provided
        /// by ReadLock(). In contentious environments this WILL result in
        /// starvation and you WILL die in a fire. DO NOT DO THIS. Use
        /// ReadLock() instead.
        [[nodiscard]] ALWAYS_INLINE bool TryReadLock()
        {
            U64 x = m_x.load(std::memory_order_relaxed);
            const U64 rw = x & (kSpin - 1);
            const U64 spinningWriter = x & kSpin;
            const bool writerPrecedence = rw && spinningWriter;
            if (LIKELY((rw < kWrite - 1) & !writerPrecedence))
            {
                return m_x.compare_exchange_strong(
                    x, spinningWriter | (rw + kRead), std::memory_order_acquire,
                    std::memory_order_relaxed);
            }
            else
            {
                return false;
            }
        }
        /// Check if the lock is available to lock for write.
        ///
        /// This is rarely what you're looking for, but it can be useful for
        /// "lock asserts": situations where you need mutual exclusion but DO
        /// NOT want to spin, and have designed your system so you should never
        /// have to - that is, mutual exclusion is supposed to be guaranteed by
        /// other means. In such cases, it can be useful to merely check that
        /// the lock is available, without actually taking it, and die if it
        /// isn't, because that indicates a logic error in whatever mechanism
        /// was supposed to ensure mutual exclusion by other means.
        [[nodiscard]] ALWAYS_INLINE bool CanWriteLock() const
        {
            return !(m_x.load(std::memory_order_relaxed) & (kSpin - 1));
        }
        /// Check if the lock is available to lock for read.
        ///
        /// This is rarely what you're looking for, but it can be useful for
        /// "lock asserts": situations where you need mutual exclusion but DO
        /// NOT want to spin, and have designed your system so you should never
        /// have to - that is, mutual exclusion is supposed to be guaranteed by
        /// other means. In such cases, it can be useful to merely check that
        /// the lock is available, without actually taking it, and die if it
        /// isn't, because that indicates a logic error in whatever mechanism
        /// was supposed to ensure mutual exclusion by other means.
        [[nodiscard]] ALWAYS_INLINE bool CanReadLock() const
        {
            U64 x = m_x.load(std::memory_order_relaxed);
            const U64 rw = x & (kSpin - 1);
            const U64 spinningWriter = x & kSpin;
            const bool writerPrecedence = rw && spinningWriter;
            return LIKELY((rw < kWrite - 1) & !writerPrecedence);
        }

    private:
        /// Storage.
        std::atomic<U64> m_x;
    };
    /// Janitorial wrapper that takes a write lock on construction and unlocks
    /// it on destruction (typically on scope exit).
    class WriteLockJanitor
    {
    public:
        /// Constructor.
        ALWAYS_INLINE WriteLockJanitor(Lock *__restrict pLock) : m_lock(*pLock)
        {
            pLock->WriteLock();
        }
        /// Constructor.
        ALWAYS_INLINE WriteLockJanitor(Lock &__restrict lock) : m_lock(lock)
        {
            lock.WriteLock();
        }
        /// Destructor.
        ALWAYS_INLINE ~WriteLockJanitor() { m_lock.WriteUnlock(); }

    private:
        /// Storage.
        Lock &__restrict m_lock;
    };
/// Important: ensures that WriteLockJanitor{lock}; (forgetting to assign to a
/// named variable) fails to compile. This would otherwise construct a temporary
/// and then immediately destruct it, locking and then immediately unlocking,
/// causing a nasty bug in which the critical section this janitor was supposed
/// to protect is not actually protected.
#define WriteLockJanitor class WriteLockJanitor
    /// Janitorial wrapper that takes a read lock on construction and unlocks it
    /// on destruction (typically on scope exit).
    class ReadLockJanitor
    {
    public:
        /// Constructor.
        ALWAYS_INLINE ReadLockJanitor(Lock *__restrict pLock) : m_lock(*pLock)
        {
            pLock->ReadLock();
        }
        /// Constructor.
        ALWAYS_INLINE ReadLockJanitor(Lock &__restrict lock) : m_lock(lock)
        {
            lock.ReadLock();
        }
        /// Destructor.
        ALWAYS_INLINE ~ReadLockJanitor() { m_lock.ReadUnlock(); }

    private:
        /// Storage.
        Lock &__restrict m_lock;
    };
/// Important: ensures that ReadLockJanitor{lock}; (forgetting to assign to a
/// named variable) fails to compile. This would otherwise construct a temporary
/// and then immediately destruct it, locking and then immediately unlocking,
/// causing a nasty bug in which the critical section this janitor was supposed
/// to protect is not actually protected.
#define ReadLockJanitor class ReadLockJanitor
} // namespace Drone::Sat