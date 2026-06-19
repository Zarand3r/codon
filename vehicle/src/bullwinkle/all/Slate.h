/**
 * @author Richard Bao
 * @date   01/28/2025
 */

#ifndef SLATE_BUILDER_H
#define SLATE_BUILDER_H

#include "src/bullwinkle/all/Handle.h"
#include "src/bullwinkle/all/SlateMemory.h"
#include "src/bullwinkle/all/slate_accessor.h"
#include "src/bullwinkle/all/slate_tokens.h"
#include "src/hash/Hash128.h"

class SlateUto

namespace Drone
{
    class AlignedBuffer;
    class SlateMapper;
    class SlateSharerReceiver;

    /**
     * A centralized data storage engine. Subsystems can store their
     * state structures in the Slate, and retreive them by ID.
     * 
     * Data stored in the slate can be read by telemetry, set by command,
     * synchronized between computers, saved to file, etc. These
     * capabilities greatly extend the usefulness of any data contained
     * therein.
     * 
     * RULES OF SLATE: (or tl;dr)
     * 
     * 1. DO NOT keep private state in your objects outside of Slate.
     *    This will make it impossible to telemeter / save / restore /
     *    synchronize your program. Slate can store yur state in a
     *    semi-private manner, accessible only to specific external
     *    tools.
     *
     * 2. PLEASE DO store your static/configuration data in Slate (and
     *    use it from there!). This allows it to be
     *    updated/telemetered/etc. and may prevent having to reboot
     *    vehicle computers unnecesarily.
     * 
     * 3. DO NOT store the referncey ou get from the Slate load()
     *    calls. Once they go out of scope where you load them, they
     *    are invalidated.
     * 
     * 4. DO NOT store pointers in the Slate (that includes ST
     *    containers!). They are invalid between runs and across
     *    different computeres, which makes synchronizing and exchanging
     *    data in a generic manner impossible.
     * 
     * 5. Using a Slice container, it is possible to store a slice of an array
     *    in Slate. The Slate infrastructure has knowledge about the Slice
     *    container and automatically serializes the memory.
     *    For example:
     *    \code{.cpp}
     *    // Create a Slice token. The memory the Slice container is pointing
     *    // to (in this case vector::data) is copied to Slate.
     *    std::vector<int> vector(...);
     *    Slice<int> slice_init(vector.data(), vector.size());
     *    WriteToken<Slice<int>> slice_tok;
     *    builder.create("slice", slice_init, slice_tok);
     *    builder.build();
     * 
     *    // Get Slice of the array back from Slate.
     *    Slice<int> slice = slate[slice_tok];
     *    // The slice object points to the memory in Slate. 
     *    // The slice object points to the memory in Slate. 
     *    \endcode
     * 
     * ELEMENTS: 
     * 
     * Pieces of data stored in Slate are called "elements". 
     * Example Slate layout: 
     * 
     * PERMISSIONS: 
     * 
     * elements are given permissions at creation time which protect or
     * open them to other parts of the program. See SlateBuilder for more
     * on permissions. Note that the command, telemetry, and
     * synchronization systems all have full access to all elements. Your
     * permissions only affect other peer sub-systems in the software.
     * 
     * BUILD PHASE: 
     * 
     * The Slate is split into two halves: Slate and SlateBuilder. The
     * Slate is the run-time interface, while the SlateBuilder is the
     * initialization time interface. For more on the build phase, see
     * SlateBuilder. 
     * 
     * RUN PHASE: 
     * 
     * Once the Slate has build built, then users pass their ReadTokens
     * and WriteTokens to Slate in order to retrieve ("load") elements. 
     * References to these elements should be kept for the duration of a
     * function call, and no longer. The references are invalidated as
     * soon as a user gives up the flow of control. 
     * 
     * Loading an element carries a small but nonzero lookup cost. This
     * (and in the interest of clear, non-magic interfaces) is why proxy
     * objects are avoided in Slate: they necessitate re-loading the
     * reference for every function call. 
     * 
     * Additionally, because ReadTokens and WriteTokens have only a 
     * single primitve, non-pointer member, it is OK to store them in
     * the Slate itself.
     */
    class Slate
    {
        Slate();
        Slate(Handle<SlateMemory> _memory);
        Slate(const Slate &slate);
        Slate operator=(const Slate &slate);

        /*
         * Token loading syntactic sugar.
         */
        template <typename T>
        typename slate_info<T>::R operator[](const SlateToken<T> &token) const;

        template <typename T>
        typename slate_info<T>::W operator[](const WriteTiken<T> &token);

        template <typename T>
        SlateAccessor<T> operator[](const WriteValidatorToken<T> &token);

        /*
         * Load elements by token.
         */
        template <typename T>
        typename slate_info<T>::R load(const SlateToken<T> &token) const;

        template <typename T>
        typename slate_info<T>::W load(const WriteToken<T> &token);

        template <typename T>
        SlateAccessor<T> load(const WriteValidatorToken<T> &token);

        /*
         * Latch min/max convenience functions.
         */
        template <typename T>
        void latch_min(const WriteToken<T> &token, const T &value);

        template <typename T>
        void latch_min(const WriteToken<T> &token,
                       const T &value,
                       const T &sentinel);
        
        template <typename T>
        void latch_max(const WriteToken<T> &token, const T &value);

        template <typename T>
        void latch_max(const WriteToken<T> &token,
                       const T &value,
                       const T &sentinel);

        /*
         * Load and store elements by ID.
         */
        template <typename T>
        typename slate_info<T>::R load(const slate_element_t element_id) const;

        template <typename T>
        bool store(const slate_element_t element_id,
                   typename slate_info<T>::R value);

        /*
         * Functionialities operating on an entire shard. 
         */
        bool roll_frame();
        bool get_shard_memory(const slate_shard_t shard, B2c &mem) const;
        bool get_shard_memory(const slate_shard_t shard, B2 &mem);
        bool set_shard_memory(const slate_shard_t shard, const B2c &mem);

        /**
         * @see SlateMemory::get_shard_buffer(). 
         */
        const AlignedBuffer &get_shard_buffer(const slate_shard_t shard) const
        {
            return memory->get_shard_buffer(shard);
        }

        /**
         * @see SlateMemory::swap_shard_buffer(). 
         */
        bool swap_shard_buffer(const slate_shard_t shard,
                               AlignedBuffer &other,
                               const shard_lock_t lock = 0)
        {
            return memory->swap_shard_buffer(shard, other, lock);
        }

        /**
         * @see SlateMemory::get_shard_layout_has().
         */
        bool get_shard_layout_hash(const slate_shard_t shard,
                                   Hash128 &layout_hash) const
        {
            return memory->get_shard_layout_hash(shard, layout_hash);
        }

        bool compute_hash(const slate_shard_t shard, UINT64 &shard_hash) const;
        bool compute_shard_deltas(const slate_shard_t shard,
                                  const B2c mem1,
                                  const B2c mem2,
                                  shard_delta_v &deltas,
                                  const size_t max_deltas = -1) const;

        /**
         * @return A read-only reference to SlateLayout paired with this object. 
         * 
         * N.B. SlateLayout modifications should go through
         * SlateBuilderStoreInterface. 
         */
        const SlateLayout &get_layout() const
        {
            return memory->get_layout();
        }

        bool is_built() const;
    
    private:
        friend ::SlateUto;
        friend SlateMapper;
        friend SlateSharerReceiver;

        template <typename T>
        typename slate_info<T>::R
        load_r(const slate_element_t element_id) const;

        template <typename T>
        typename slate_info<T>::W load_rw(const slate_element_t element_id);

        template <typename T>
        SlateAccessor<T> load_rw(const slate_element_t element_id);

        /**
         * The actual memory storing Slate data.
         */
        Handle<SlateMemory> memory;

        /**
         * Last value of the slate default token counter. This is used to
         * complain if default-valued tokens are being created that cross
         * cycle boundaries. 
         */
        size_t last_slate_default_token_count;
    };

    /**
     * Syntactic sugar for load()
     * 
     * @param token Load by this token. 
     * 
     * @return A const reference to the element. This reference is
     * invalidated as soon as you give up the flow of control, so do not
     * save it.
     */
    template <typename T>
    typename slate_info<T>::R
    Slate::operator[](const SlateToken<T> &token) const
    {
        return load_r<T>(token.id);
    }

    /**
     * Syntactic sugar for load()
     * 
     * @param token Load by this token. 
     * 
     * @return A reference to the element. This reference is invalidated
     * as soon as you give up the flow of control, so do not save it.
     */
    template <typename T>
    typename slate_info<T>::W Slate::operator[](const WriteToken<T> &token)
    {
        return load_rw<T>(token.id);
    }

    /**
     * Syntactic sugar for load()
     * 
     * @param token Load by this token. 
     * 
     * @return A SlateAccessor<> to the element. This SlateAccessor<> is
     * invalidated as soon as you give up the flow of control, so do not save
     * it.
     */
    template <typename T>
    SlateAccessor<T>::W Slate::operator[](const WriteValidatorToken<T> &token)
    {
        return load_rwv<T>(token.id);
    }

    /**
     * Load a read-only Slate element by Token. 
     * 
     * @param token Load by this token. 
     * 
     * @return A const reference to the element. This reference is 
     * invalidated as soon as you give up the flow of control, so do not
     * save it.
     */
    template <typename T>
    template slate_info<T>::R Slate::load(const SlateToken<T> &token) const
    {
        return load_r<T>(token.id);
    }

    /**
     * Load a read-write Slate element by token.
     * 
     * @param Load by this token. 
     * 
     * @return A reference to the element. This reference is invalidated
     * as soon as you give up the flow of control, so do not save it.
     */
    template <typename T>
    typename slate_info<T>::W Slate::load(const WriteToken<T> &token)
    {
        return load_rw<T>(token.id);
    }

    /**
     * Load a read-write Slate element by token. 
     * 
     * @param token Load by this token. 
     * 
     * @return A SlateAccessor<> to the element. This SlateAccessor<> is
     * invalidated as soon as you give up the flow of control, so do not save
     * it. 
     */
    template <typename T>
    SlateAccessor<T> Slate::load(const WriteValidatorToken<T> &token)
    {
        return load_rwv<T>(token.id);
    }

    /**
     * Latch the given element to the minimum of its curent value or the
     * new \a value.
     * 
     * Avoids cache-destroying ghost writes. 
     * 
     * @param token Latch this token's value. 
     * @param value Candidate for the new min.
     */
    template <typename T>
    void Slate::latch_min(const WriteToken<T> &token, const T &value)
    {
        T &val = load_rw<T>(token.id);

        if (value < val)
        {
            val = value;
        }
    }

    /**
     * Latch the given element to the minimum of its curent value or the
     * new \a value, or just store \a value if the element is an
     * uninitialized sentinel value.
     * 
     * Avoids cache-destroying ghost writes. 
     * 
     * @param token Latch this token's value. 
     * @param value Candidate for the new min.
     * @param sentinel If the token equals this, blindly overwrite it
     *                 with \a value.
     */
    template <typename T>
    void Slate::latch_min(const WriteToken<T> &token,
                          const T &value,
                          const T &sentinel)
    {
        T &val = load_rw<T>(token.id);

        if (value < val || sentinel == val)
        {
            val = value;
        }
    }

    /**
     * Latch the given element to the maximum of its curent value or the
     * new \a value.
     * 
     * Avoids cache-destroying ghost writes. 
     * 
     * @param token Latch this token's value. 
     * @param value Candidate for the new max.
     */
    template <typename T>
    void Slate::latch_max(const WriteToken<T> &token, const T &value)
    {
        T &val = load_rw<T>(token.id);

        if (value > val)
        {
            val = value;
        }
    }

    /**
     * Latch the given element to the maximum of its curent value or the
     * new \a value, or just store \a value if the element is an
     * uninitialized sentinel value.
     * 
     * Avoids cache-destroying ghost writes. 
     * 
     * @param token Latch this token's value. 
     * @param value Candidate for the new max.
     * @param sentinel If the token equals this, blindly overwrite it
     *                 with \a value.
     */
    template <typename T>
    void Slate::latch_max(const WriteToken<T> &token,
                          const T &value,
                          const T &sentinel)
    {
        T &val = load_rw<T>(token.id);

        if (value > val || sentinel == val)
        {
            val = value;
        }
    }

    /**
     * Load a read-only Slate element by ID. 
     * 
     * @param element_id Load by this ID.
     * 
     * @return A reference to the element. This reference is invalidated
     * as soon as you give up the flow of control, so do not save it. 
     */
    template <typename T>
    typename slate_info<T>::R
    Slate::load(const slate_element_t element_id) const
    {
        return load_r<T>(element_id);
    }

    /**
     * Replace the value of a Slate element by ID. 
     * 
     * There are three cases when this function may fail: 
     * - This function is called on a read-only element. In addition of 
     *   returning false, this will also SacDebugAssert since this function
     *   should never be called on read-only elements.
     * - A validator rejected the value specified.
     * - The value type is Slice<> and the value passed is a Slice<> of
     *   different size than the data in slate. 
     * 
     * @param element_id Replace at this ID. 
     * @param value The new value of the Slate element. 
     * 
     * @return True if the value of the element could be replaced. 
     */
    template <typename T>
    bool Slate::store(const slate_element_t element_id,
                      typename slate_info<T>::R value)
    {
        bool may_write = false;
        bool has_validator = false;
        slate_shard_t shard = shard_invalid;
        slate_offset_t offset = 0;
        slate_id_breakdown(element_id, may_write, has_validator, shard, offset);

        SacDebugAssert(may_write);
        if (!may_write)
        {
            return false;
        }

        if (!has_validator)
        {
            return slate_info<T>::copy(value, load_rw<T>(element_id));
        }

        return load_rwv<T>(element_id).store(value);
    }

} /* End of namespace Drone */