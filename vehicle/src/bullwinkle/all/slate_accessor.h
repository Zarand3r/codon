/**
 * @author Richard Bao
 * @date   01/28/2025
 */

#ifndef SLATE_ACCESSOR_H
#define SLATE_ACCESSOR_H

#include "src/bullwinkle/all/Handle.h"
#include "src/bullwinkle/all/Signal.h"
#include "src/bullwinkle/all/slate_info.h"

#include <functional>
#include <utility>

class SlateAccessorUto;
class SlateBuilderUto;

namespace Drone
{
    class SlateBuilder;

    /**
     * Type-erased base for all validators. Polymorphic (virtual dtor) so a
     * `slate_validator_t`'s `Handle<SlateValidator>` can be dynamically down-cast
     * back to the concrete `SlateTypedValidator<T>` at bind/store time (via
     * `Handle::assign_casted`, which is `std::dynamic_pointer_cast`). This design
     * is dictated by the consumer (SlateBuilder.h binds validators exactly so).
     */
    class SlateValidator
    {
    public:
        virtual ~SlateValidator() {}
    };

    /**
     * Typed validator interface. `validate` receives the proposed value and a
     * reference to the element's storage; on success it writes the accepted value
     * into `val` and returns true, on rejection it returns false and leaves `val`
     * unchanged.
     */
    template <typename T>
    class SlateTypedValidator : public SlateValidator
    {
    public:
        virtual bool validate(const T &new_val, T &val) const = 0;
    };

    /***********************************************************************
     * 
     * SlateAccessor & validators API. 
     * 
     * During slate element creation, the code that creates a new slate element
     * can specify a value validation function. Slate value validation functions
     * are called at the time a caller attemptse to write to a slate element. 
     * 
     * If a validation function is specified for a slate element, it is only
     * possible to bind to this element for writing using a WriteValidatorToken.
     * 
     * For example:
     * \code{.cpp}
     * // Create the element with a validation function. 
     * SlateBuilder builder;
     * 
     * ReadToken<int> reader_tok;
     * FswAbortIfNot(builder.create("test_enum",
     *                               test_value0,
     *                               shard_sync,
     *                               slate_read_write,
     * 
     *                               // The validation function
     *                               enum_validator<test_enum_t>(),
     *                               reader_tok);
     *               false);
     * 
     * // Bind to the element. 
     * WriteValidatorToken<int> writer_tok;
     * FswAbortIfNot(builder.bind("test_enum", writer_tok), false);
     * \endcode
     * 
     * Post initialization, it is possible to set the value of the element using
     * a proxy objectl SlateAccessor. When using WriteValidatorToken, slate
     * returns SlateAccessor proxy objects instead of a direct reference to
     * elements memory. 
     * 
     * SlateAccessor provides both setters and getters. So it is possible to
     * both read or write to the element. When writing to the element, the
     * validation function is invoked. If it fails, the slate element value is
     * unchanged. 
     * 
     * For example: 
     * \code{.cpp}
     * // Explicitly use the proxy object. 
     * SlateAccessor<int> proxy = slate[writer_tok];
     * proxy = test_value1;
     * FswAbortIfNeq(proxy, test_value1, false);
     * 
     * // Common scenario: do not explicitly use the proxy object. 
     * slate[writer_tok] = test_value1;
     * FswAbortIfNeq(slate[writer_tok], test_value1, false);
     * \endcode
     * 
     * If a caller needs to know that writing to the element failed, it should
     * use the SlateAccessor store() method instead: 
     * \code{.cpp}
     * FswAbortIf(slate[writer_tok].store(18), false);
     * 
     * // The value wasn't changed. 
     * FswAbortIfNeq(slate[writer_tok], test_value1, false);
     * \endcode
     * 
     * The big advantage of using a validation function is that the dispatched
     * code is guaranteed that the slate element value is correct. This
     * minimizes edge cases to handle.
     * 
     * For example, a dispatch() method that reads from reader_tok, which was
     * created using enum_validator<test_enum_t>(): 
     * \code{.cpp}
     * void dispatch()
     * {
     *     switch (slate[reader_tok])
     *     {
     *     case test_value0: do_stuff(); break;
     *     case test_value1: do_stuff(); break;
     *     case test_value2: do_stuff(); break;
     *     default:
     *         // This is not possible. The slate element can't be set to this
     *         // value. No special handling necessary.
     *         FswAssert(0);
     *     }
     * }
     * \endcode
     * 
     * Two types of validators are provided: 
     * - Enum validators. 
     * - Member validators.
     * 
     * Enum validators work for any auto-enums, and will return a failure if the
     * new value is not one of the possible enumeration values for an enum type.
     * 
     * For example, create a slate element with an enum validator:
     * \code{.cpp}
     * ReadToken<int> reader_tok;
     * FswAbortIfNot(builder.create("test_enum",
     *                               test_value0,
     *                               shard_sync,
     *                               slate_read_write,
     *                               enum_validator<test_enum_t>(),
     *                               reader_tok),
     *               false);
     * \endcode
     * 
     * Function and member validators provide an easy mechanism for implementing
     * a custom validation function, either as a free function or a member
     * method. A custom validation function must have the following signature:
     * 
     * \code{.cpp}
     * // T is the type of the element to validate. 
     * // new_val: the new value for the slate element. 
     * // val: reference to the slate element to update. 
     * 
     * bool my_function(const T &new_val, T &val);
     * 
     * bool MyClass::validate(const T &new_val, T &val) const;
     * \endcode
     *
     **********************************************************************/

    /**
     * Concrete validator wrapping a std::function. The functor is heap-held (via
     * the Handle) at *build* time only — validators are attached during Slate
     * construction, never on the runtime hot path (a store's cost is one virtual
     * call, no allocation).
     */
    template <typename T>
    class SlateFunctionValidator : public SlateTypedValidator<T>
    {
    public:
        typedef std::function<bool(const T &new_val, T &val)> fn_t;

        explicit SlateFunctionValidator(fn_t fn) : fn(std::move(fn)) {}

        bool validate(const T &new_val, T &val) const override
        {
            return fn ? fn(new_val, val) : false;
        }

    private:
        fn_t fn;
    };

    /**
     * Type-erased validator handle stored per element. Default-constructed it is a
     * "no-op" (no validator attached). Copyable/returnable by value (shares the
     * underlying validator through the Handle).
     */
    struct slate_validator_t
    {
        Handle<SlateValidator> internal{};

        /** True if no validator is attached. */
        bool is_noop() const { return !internal; }

        /** True if a usable validator is attached. */
        bool is_usable() const { return internal.operator bool(); }
    };

    /**
     * Build a validator from a function object `bool(const T& new_val, T& val)`.
     * This is the general factory; specific factories (enum/member validators) can
     * be layered on top when a call site needs them.
     */
    template <typename T>
    inline slate_validator_t
    function_validator(typename SlateFunctionValidator<T>::fn_t fn)
    {
        slate_validator_t v;
        v.internal.assume_ownership(new SlateFunctionValidator<T>(std::move(fn)));
        return v;
    }

    /**
     * Write proxy returned when accessing a validated element. Reading converts to
     * the element value; writing (operator= / store) runs the validator, so the
     * element only changes if validation passes. `store` returns false on rejection
     * (value unchanged); operator= is the fire-and-forget form.
     */
    template <typename T>
    class SlateAccessor
    {
    public:
        typedef typename slate_info<T>::R R;

        SlateAccessor(void *mem, slate_validator_t validator)
            : value_ptr(static_cast<T *>(mem)), validator(std::move(validator))
        {}

        /** Read: convert to the element's value. */
        operator R() const { return *value_ptr; }

        /** Get a const reference to the current value (no write). */
        R get() const { return *value_ptr; }

        /**
         * Validated write. Down-casts the type-erased validator to this element's
         * type and runs it; the element is updated only if it accepts.
         *
         * @return True if the value was accepted and stored.
         */
        bool store(const T &new_val)
        {
            Handle<SlateTypedValidator<T>> typed;
            if (!typed.assign_casted(validator.internal))
            {
                return false; /* wrong type or no validator */
            }
            return typed->validate(new_val, *value_ptr);
        }

        /** Convenience write; ignores the accept/reject result. */
        SlateAccessor &operator=(const T &new_val)
        {
            store(new_val);
            return *this;
        }

    private:
        T *value_ptr;
        slate_validator_t validator;
    };

} /* end namespace Drone */

#endif /* SLATE_ACCESSOR_H */