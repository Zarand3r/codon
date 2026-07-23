/**
 * @author Richard Bao
 * @date   01/28/2025
 */

#ifndef HANDLE_H
#define HANDLE_H

#include "src/bullwinkle/all/core/fsw.h"

#include <functional>
#include <memory>

namespace Drone
{
    /**
     * The Handle class is a smart pointer which assumes the ownership of an
     * object and ensures that it is deleted when the last reference is deleted.
     *
     * It is a thin wrapper around std::shared_ptr.
     *
     * Hotsync analyzer modifies the interface of Handle to try to ensure that
     * objects pointed to by member Handle objects cannot be modified from
     * RUNTIME methods. This is the main reason we require using Handles rather
     * than std::unique_ptr and std::shared_ptr directly.
     */
    template <typename T>
    class Handle
    {
        template <typename U>
        friend class Handle;

    public:
        /**
         * Create an empty Handle.
         */
        Handle() : internal() {}

        /**
         * Create a Handle which assumes ownership of the supplied pointer.
         *
         * @tparam U The type of the object being provided. U* can be implicitly
                     converted to T*.
         * @param ptr The pointer to the object to assume ownership of.
         */
        template <typename U>
        explicit Handle(U *const ptr) : internal(ptr)
        {}

        /**
         * Create a Handle which shares ownership with the supplied Handle.
         *
         * Note: if U == T, then the implicit copy or move constructor is used
         * instead.
         *
         * @tparam U The type of the object being shared. U* can be implicitly
                     converted to T*.
         * @param h The handle to share ownership with.
         */
        template <class U>
        Handle(const Handle<U> &h) : internal(h.internal)
        {}

        /**
         * Create a handle which initializes the managed object using the
         * supplied arguments.
         *
         * @tparam args The arguments to forward to the constructor for T.
         *
         * @return The handle that was created using the arguments.
         */
        template <typename... Ts>
        static Handle create(Ts &&...args)
        {
            Handle res;
            FswAbortIfNot(res.assume_ownership(
                              new (std::nothrow) T(std::forward<Ts>(args)...)),
                          Handle());

            return res;
        }

        /**
         * Take over the ownership of the supplied object. The object will be
         * deleted when the last reference is gone.
         *
         * @tparam U The type of the object being provided. U* can be implicitly
         *           converted to T*.
         * @param ptr The object to take over.
         *
         * @return Returns *this.
         */
        template <typename U>
        Handle &assume_ownership(U *const ptr)
        {
            internal.reset(ptr);

            return *this;
        }

        /**
         * Drop the current reference (deleting the object if needed) and become
         * empty.
         *
         * @return Returns *this.
         */
        Handle &clear()
        {
            internal.reset();

            return *this;
        }

        /**
         * Check if this is a non-empty Handle.
         *
         * @note This conversion operator needs to be explicit so the compiler
         *       doesn't attempt to convert a Handle to another type via a bool
         *       without our knowledge.
         *
         * @return True if this is a non-empty Handle.
         */
        explicit operator bool() const { return internal.operator bool(); }

        /**
         * Return true if this Handle is unique.
         *
         * A Handle is unique if it is the sole owner of the underlying object
         * and it is not an empty Handle.
         *
         * @return True if this Handle is unique.
         */
        bool is_unique() const { return internal.use_count() == 1; }

        /**
         * Returns a pointer to the underlying object.
         *
         * @return A pointer to the underlying object.
         */
        T *get() const { return internal.get(); }

        /**
         * Dereference operator.
         *
         * For Handle<void>, return void.
         *
         * @return A reference to the underlying object.
         */
        typename std::add_lvalue_reference<T>::type operator*() const
        {
            /*
             * This code is in the hot path and asserting `internal` here is too
             * expensive. We still want to get a nice stack trace in debug
             * builds.
             */
            FswDebugAssert(internal);

            return *internal;
        }

        /**
         * Member access operator.
         *
         * @return A pointer to the underlying object.
         */
        T *operator->() const
        {
            /*
             * This code is in the hot path and asserting `internal` here is too
             * expensive. We still want to get a nice stack trace in debug
             * builds.
             */
            FswDebugAssert(internal);

            return internal.get();
        }

        /**
         * Attempt to assign a Handle of a different type to this Handle.
         *
         * This uses a dynamic cast to test if the conversion if possible.
         * If it is not possible, this Handle will become empty.
         *
         * @tparam U The type of the other Handle.
         * @param h The other Handle.
         *
         * @return True if the conversion was successful.
         */
        template <class U>
        bool assign_casted(const Handle<U> &h)
        {
            internal = std::dynamic_pointer_cast<T>(h.internal);

            return internal.operator bool();
        }

    private:
        /**
         * A pointer to the underlying object.
         */
        std::shared_ptr<T> internal;
    };

    /**
     * @{
     * @name Handle comparison operators.
     *
     * Ordering operators define a total order on all pointers, same as
     * std::shared_ptr. This is unlike raw pointers, which only define an order
     * within the same array.
     */
    template <typename T1, typename T2>
    inline bool operator==(const Handle<T1> &l, const Handle<T2> &r)
    {
        return l.get() == r.get();
    }

    template <typename T>
    inline bool operator==(const T *const l_ptr, const Handle<T> &r)
    {
        return l_ptr == r.get();
    }

    template <typename T>
    inline bool operator==(const Handle<T> &l, const T *const r_ptr)
    {
        return l.get() == r_ptr;
    }

    template <typename T1, typename T2>
    inline bool operator!=(const Handle<T1> &l, const Handle<T2> &r)
    {
        return l.get() != r.get();
    }

    template <typename T>
    inline bool operator!=(const T *const l_ptr, const Handle<T> &r)
    {
        return l_ptr != r.get();
    }

    template <typename T>
    inline bool operator!=(const Handle<T> &l, const T *const r_ptr)
    {
        return l.get() != r_ptr;
    }

    template <typename T1, typename T2>
    inline bool operator<(const Handle<T1> &l, const Handle<T2> &r)
    {
        using CT = typename std::common_type<T1 *, T2 *>::type;
        return std::less<CT>()(l.get(), r.get());
    }

    template <typename T>
    inline bool operator<(const T *l_ptr, const Handle<T> &r)
    {
        return std::less<const T *>()(l_ptr, r.get());
    }

    template <typename T>
    inline bool operator<(const Handle<T> &l, const T *r_ptr)
    {
        return std::less<const T *>()(l.get(), r_ptr);
    }

    template <typename T1, typename T2>
    inline bool operator<=(const Handle<T1> &l, const Handle<T2> &r)
    {
        using CT = typename std::common_type<T1 *, T2 *>::type;
        return std::less_equal<CT>()(l.get(), r.get());
    }

    template <typename T>
    inline bool operator<=(const T *l_ptr, const Handle<T> &r)
    {
        return std::less_equal<const T *>()(l_ptr, r.get());
    }

    template <typename T>
    inline bool operator<=(const Handle<T> &l, const T *r_ptr)
    {
        return std::less_equal<const T *>()(l.get(), r_ptr);
    }

    template <typename T1, typename T2>
    inline bool operator>(const Handle<T1> &l, const Handle<T2> &r)
    {
        using CT = typename std::common_type<T1 *, T2 *>::type;
        return std::greater<CT>()(l.get(), r.get());
    }

    template <typename T>
    inline bool operator>(const T *l_ptr, const Handle<T> &r)
    {
        return std::greater<const T *>()(l_ptr, r.get());
    }

    template <typename T>
    inline bool operator>(const Handle<T> &l, const T *r_ptr)
    {
        return std::greater<const T *>()(l.get(), r_ptr);
    }

    template <typename T1, typename T2>
    inline bool operator>=(const Handle<T1> &l, const Handle<T2> &r)
    {
        using CT = typename std::common_type<T1 *, T2 *>::type;
        return std::greater_equal<CT>()(l.get(), r.get());
    }

    template <typename T>
    inline bool operator>=(const T *l_ptr, const Handle<T> &r)
    {
        return std::greater_equal<const T *>()(l_ptr, r.get());
    }

    template <typename T>
    inline bool operator>=(const Handle<T> &l, const T *r_ptr)
    {
        return std::greater_equal<const T *>()(l.get(), r_ptr);
    }
    /*
     * @}
     */


} /* end namespace Drone */

#endif /* HANDLE_H */