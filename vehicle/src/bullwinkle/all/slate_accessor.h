/**
 * @author Richard Bao
 * @date   01/28/2025
 */

#ifndef SLATE_ACCESSOR_H
#define SLATE_ACCESSOR_H

#include "src/bullwinkle/all/Handle.h"
#include "src/bullwinkle/all/Signal.h"
#include "src/bullwinkle/all/core/slate_info.h"

class SlateAccessorUto;
class SlateBuilderUto;

namespace Drone
{
    class SlateBuilder;

    template<typename T>
    class SlateTypedValidator;

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
     * SacAbortIfNot(builder.create("test_enum",
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
     * SacAbortIfNot(builder.bind("test_enum", writer_tok), false);
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
     * SacAbortIfNeq(proxy, test_value1, false);
     * 
     * // Common scenario: do not explicitly use the proxy object. 
     * slate[writer_tok] = test_value1;
     * SacAbortIfNeq(slate[writer_tok], test_value1, false);
     * \endcode
     * 
     * If a caller needs to know that writing to the element failed, it should
     * use the SlateAccessor store() method instead: 
     * \code{.cpp}
     * SacAbortIf(slate[writer_tok].store(18), false);
     * 
     * // The value wasn't changed. 
     * SacAbortIfNeq(slate[writer_tok], test_value1, false);
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
     *         SacAssert(0);
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
     * SacAbortIfNot(builder.create("test_enum",
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
}