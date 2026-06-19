/**
 * @author Richard Bao
 * @date   01/28/2025
 */

#ifndef SLATE_TOKENS_H
#define SLATE_TOKENS_H

#include "src/bullwinkle/all/slate_info.h"

#include <map>
#include <set>
#include <string>

/*
 * We want to make the unit test a friend of the tokens, so we
 * forward-declare it here.
 */
class SlateBuilderUto;
class SlateTokensUto;

namespace Drone
{
    class Slate;
    class SlateBuilder;
    class SlateMapper;
    class SlatePubSub;

    typedef std::map<slate_element_t, std::string> slate_element_str_m;

    /**
     * Common functionalities for all types of token.
     *
     * @tparam T The element data type.
     */
    template <typename T>
    class SlateToken
    {
    protected:
        SlateToken();
        SlateToken(const SlateToken<T> &b);
        ~SlateToken();

        slate_type_t get_type_id() const;

        void set(const SlateToken<T> &b);

    private:
        /*
         * Only special slate classes should be able to modify the ID.
         */
        friend class Slate;
        friend class SlateBuilder;
        friend class SlateMapper;
        friend class SlatePubSub;
        friend class ::SlateBuilderUto;
        friend class ::SlateTokensUto;

        /**
         * The element ID that we're bound to.
         */
        slate_element_t id;
    };

    /**
     * Slate token access rights flags.
     */
    enum slate_token_access_t
    {
        /**
         * Read access.
         */
        slate_token_access_read = 1 << 0,

        /**
         * Write access.
         */
        slate_token_access_write = 1 << 1,

        /**
         * Does on-write validation.
         */
        slate_token_access_validation = 1 << 2
    };

    /**
     * A common utility for slate element tokens with different access rights.
     *
     * @tparam T The element data type.
     * @tparam Access A combination of slate_token_access_t values.
     */
    template <typename T, int Access>
    class SlateAccessToken: public SlateToken<T>
    {
    public:
        /**
         * True if this token has read access.
         */
        static const bool can_read = !!(Access & slate_token_access_read);

        /**
         * True if this token has write access.
         */
        static const bool can_write = !!(Access & slate_token_access_write);

        /**
         * True if this token has write access to elements requiring validation.
         */
        static const bool can_validate =
            !!(Access & slate_token_access_validation);

        /*
         * A token must be able to read, write or both.
         */
        static_assert(can_read || can_write);

        /*
         * A token which can validate must be able to write.
         */
        static_assert(!can_validate || can_write);

        /**
         * Constructor.
         */
        SlateAccessToken(): SlateToken<T>()
        {}

        /**
         * Copy constructor.
         *
         * @param b Copy this token.
         */
        SlateAccessToken(const SlateAccessToken &b): SlateToken<T>(b)
        {}

        /**
         * Get the type ID for this token.
         *
         * @return The type ID.
         */
        slate_type_t type_id() const
        {
            return this->get_type_id();
        }

        /**
         * Assignment operator.
         *
         * @param b Copy this token.
         *
         * @return This token.
         */
        SlateAccessToken &operator=(const SlateAccessToken &b)
        {
            this->set(b);
            return *this;
        }
    };

    /**
     * Read-only slate element token. Pass this to slate to access an
     * element.
     */
    template <typename T>
    using ReadToken = SlateAccessToken<T, slate_token_access_read>;

    /**
     * Read-write slate element token. Pass this to slate to access an
     * element.
     */
    template <typename T>
    using WriteToken =
        SlateAccessToken<T, slate_token_access_read | slate_token_access_write>;

    /**
     * Read-write slate element token with an on-write validator. Pass this to
     * slate to access an element.
     */
    template <typename T>
    using WriteValidatorToken =
        SlateAccessToken<T,
                         slate_token_access_read | slate_token_access_write |
                             slate_token_access_validation>;

    /**
     * Interface for the token accountant so multiple accountants can be
     * 'merged'. This is useful when Slate is being shared across multiple
     * modules.
     */
    class SlateTokenAccountantInterface
    {
    public:
        /**
         * Destructor.
         */
        virtual ~SlateTokenAccountantInterface()
        {}

        /**
         * Check for any uninitialized tokens.
         *
         * @return True if all tokens were deleted or initialized.
         */
        virtual bool check_for_uninitialized_tokens() const = 0;

        /**
         * Acknowledge all outstanding IDs. Clears all valid ID entries,
         * moves the first ID to the current value, and disables the
         * accountant.
         *
         * @return True on success.
         */
        virtual bool acknowledge_and_disable() = 0;
    };

    /**
     * The token accountant is responsible for keeping track of all
     * ReadTokens and WriteTokens created at initialization time.
     * Specifically, it enables a SlateBuilder to check, at build time,
     * whether anyone is holding on to an uninitialized token that they
     * could potentially try and use at run time.
     *
     * As tokens are created, they are assigned a unique invalid ID from
     * the accountant. This ID can be validated in two ways:
     *
     *  1. If the token is deleted before being bound, it is not possible
     *     to use it anymore. Its ID is bound to an empty string in the
     *     map.
     *  2. If the token is bound to a name, the SlateBuilder which did
     *     the binding will map its ID to that name in the map.
     *
     * When build() is called, every single ID given out by the
     * accountant should be bound to some string, either empty or real.
     * IDs missing from the map are uninitialized and could crash the
     * program.
     *
     * WARNING: Once building is completed, you must call
     * acknowledge_and_disable(). This will wipe the current memory map
     * and stop any further allocations. Failure to do this will allow
     * memory to continuously leak out at run time if any tokens are
     * created.
     */
    class SlateTokenAccountant: public SlateTokenAccountantInterface
    {
    public:
        /**
         * Structure for representing a stack frame.  Used to print state about
         * the location where uninitialized tokens were allocated from.
         */
        struct StackFrame
        {
            /**
             * The context from the closest Sac macro, if any.
             */
            std::string sac_context = {};
        };

        using element_stack_frame_m = std::map<slate_element_t, StackFrame>;

        SlateTokenAccountant();

        slate_element_t generate_id();
        bool dismiss_id(const slate_element_t id);
        bool register_id(const slate_element_t id, const std::string &name);

        slate_element_t get_first_invalid_id() const;
        slate_element_t get_last_invalid_id() const;

        void enable();

        bool check_for_uninitialized_tokens() const override;
        bool acknowledge_and_disable() override;

        size_t get_valid_id_count() const
        {
            return valid_id_count;
        }

        const element_stack_frame_m &get_id_frames() const
        {
            return id_frames;
        }

    private:
        /**
         * The count of the first ID given out since the last time an ID
         * check was performed.
         */
        slate_element_t first_invalid_count;

        /**
         * The count of the next ID to give out.
         */
        slate_element_t next_invalid_count;

        /**
         * Number of tokens dismissed or registered (i.e., number of tokens
         * created that are not uninitailized).
         */
        size_t valid_id_count = 0;

        /**
         * Store the caller address and closest surrounding Sac macro for each
         * token ID.
         */
        element_stack_frame_m id_frames;
    };

    extern bool _slate_token_accountant_enabled;
    extern size_t _slate_default_token_count;
    SlateTokenAccountant &slate_token_accountant();

    namespace UnitTestUtilities
    {
        void slate_token_accountant_auto_re_enable();
    }

    namespace Deprecated15237
    {
        void slate_token_accountant_force_enable();
    }

    /**
     * Constructor.
     */
    template <typename T>
    SlateToken<T>::SlateToken(): id(slate_element_default)
    {
        /*
         * Check the enable flag before getting the accountant.
         */
        if (_slate_token_accountant_enabled)
        {
            id = slate_token_accountant().generate_id();
        }

        if (slate_element_default == id)
        {
            ++_slate_default_token_count;
        }
    }

    /**
     * Copy constructor.
     *
     * @param b Copy this token.
     */
    template <typename T>
    SlateToken<T>::SlateToken(const SlateToken<T> &b): id(slate_element_default)
    {
        ++_slate_default_token_count;
        set(b);
    }

    /**
     * Destructor.
     */
    template <typename T>
    SlateToken<T>::~SlateToken()
    {
        /*
         * Check the enable flag before getting the accountant to avoid
         * the mutex.
         */
        if (_slate_token_accountant_enabled && !slate_id_is_valid(id) &&
            slate_element_default != id)
        {
            const bool success = slate_token_accountant().dismiss_id(id);
            SacAssert(success);
        }

        if (slate_element_default == id)
        {
            --_slate_default_token_count;
        }
    }

    /**
     * Get the type ID for this token.
     *
     * @return The type ID.
     */
    template <typename T>
    slate_type_t SlateToken<T>::get_type_id() const
    {
        return slate_type_id<T>();
    }

    /**
     * Assignment operator.
     *
     * @param b Copy this token.
     *
     * @return This token.
     */
    template <typename T>
    void SlateToken<T>::set(const SlateToken<T> &b)
    {
        const slate_element_t old_id = id;

        /*
         * Check the enable flag before getting the accountant to avoid
         * the mutex.
         */
        if (_slate_token_accountant_enabled)
        {
            /*
             * If we have an invalid ID, we need to dismiss it.
             */
            if (!slate_id_is_valid(id) && slate_element_default != id)
            {
                const bool success = slate_token_accountant().dismiss_id(id);
                SacAssert(success);
            }

            /*
             * If the other ID is invalid, get ourselves a unique invalid
             * ID.
             */
            if (!slate_id_is_valid(b.id))
            {
                id = slate_token_accountant().generate_id();
            }
            else
            {
                id = b.id;
            }
        }
        else
        {
            if (!slate_id_is_valid(b.id))
            {
                id = slate_element_default;
                ++_slate_default_token_count;
            }
            else
            {
                id = b.id;
            }
        }

        if (slate_element_default == old_id)
        {
            --_slate_default_token_count;
        }
    }

} /* end namespace Drone */

#endif /* SLATE_TOKENS_H */