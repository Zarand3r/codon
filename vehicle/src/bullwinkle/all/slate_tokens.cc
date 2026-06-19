/**
 * @author Richard Bao
 * @date   01/28/2025
 */

#include "src/bullwinkle/all/slate_tokens.h"

namespace Drone
{
    /**
     * Indicates whether the static token accountant is enabled. This is
     * used as a first check to avoid getting the function-local mutex
     * in slate_token_accountant() at run time.
     *
     * Start enabled to catch any tokens created before the slate.
     *
     * EXTERNAL CODE SHOULD NOT INTERACT WITH THIS BOOLEAN.
     */
    bool _slate_token_accountant_enabled = true;

    /**
     * Used by unit-tests only. It is used to easily re-enable the slate token
     * accountants between each test.
     */
    namespace
    {
        bool _slate_token_accountant_auto_re_enable = false;
    }

    /**
     * Number of tokens created which were assigned the default token ID.
     * The slate should NEVER be allowed to build if this count is not
     * zero, since we have no way of tracking if they were registered or
     * not. This should only be possible if tokens have been created
     * after a slate was built but before another one was created.
     *
     * EXTERNAL CODE SHOULD NOT INTERACT WITH THIS VARIABLE.
     */
    size_t _slate_default_token_count = 0;

    /**
     * Constructor.
     */
    SlateTokenAccountant::SlateTokenAccountant():
        first_invalid_count(0),
        next_invalid_count(first_invalid_count),
        id_frames()
    {}

    /**
     * Returns a statically-allocated SlateTokenAccountant.
     *
     * This uses a function-local static, which has a mutex on it. Don't
     * call this at high rate during run time.
     *
     * @return SlateTokenAccountant.
     */
    SlateTokenAccountant &slate_token_accountant()
    {
        static SlateTokenAccountant acct;
        return acct;
    }

    namespace UnitTestUtilities
    {
        /**
         * Set _slate_token_accountant_auto_re_enable to true.
         */
        void slate_token_accountant_auto_re_enable()
        {
            _slate_token_accountant_auto_re_enable = true;
        }
    }

    namespace Deprecated15237
    {
        /**
         * Do not use!
         *
         * This is only provided as backward compatibility for:
         * - Fusion also makes use of this function right now. Do not remove
         *   this function before verifying Fusion do not need it anymore.
         *
         * Enable the accountant regardless of the flag
         * _slate_token_accountant_auto_re_enable.
         */
        void slate_token_accountant_force_enable()
        {
            _slate_token_accountant_enabled = true;
        }
    }

    /**
     * Generate the next unique invalid ID. If the accountant is
     * currently disabled, it will return slate_element_default.
     *
     * @return A unique invalid ID.
     */
    slate_element_t SlateTokenAccountant::generate_id()
    {
        if (!_slate_token_accountant_enabled)
        {
            return slate_element_default;
        }

        const slate_element_t id = slate_id_build_invalid(next_invalid_count++);

        StackFrame stack_frame;
        const SacStackFrame *ssf = SacStackFrame::get_current_stack_frame();
        if (ssf)
        {
            stack_frame.sac_context = ssf->file_location;
        }

        id_frames[id] = stack_frame;

        return id;
    }

    /**
     * Dismiss an invalid ID. It will no longer register as an
     * uninitialized ID.
     *
     * @param id Dismiss this ID.
     *
     * @return Always true.
     */
    bool SlateTokenAccountant::dismiss_id(const slate_element_t id)
    {
        SacAbortIfNot(register_id(id, ""), false);
        return true;
    }

    /**
     * Register an invalid ID with an element name. It will no longer
     * register as an uninitialized ID.
     *
     * @param id Register this ID.
     * @param name Register the id to this name.
     *
     * @return Failure if the ID has already been registered. Always true
     *         if the accountant is disabled.
     */
    bool SlateTokenAccountant::register_id(const slate_element_t id,
                                           const std::string &name)
    {
        SacAbortIf(slate_id_is_valid(id), false);

        /*
         * Ignore any registrations while disabled.
         */
        if (!_slate_token_accountant_enabled)
        {
            return true;
        }

        const bool is_known_id = id_frames.erase(id) == 1;

        if (!is_known_id)
        {
            /*
             * The other case, where id != slate_element_default, means that
             * there were duplicate unique invalid IDs given out. That should
             * just not be possible. We assert it out here.
             */
            SacAssert(id == slate_element_default);

            std::string name_phrase;

            if (name.empty())
            {
                name_phrase = "an uninitialized (but deleted) token";
            }
            else
            {
                name_phrase = "the token for element '" + name + "'";
            }

            SacPrefix();
            dbnprintf(500,
                      ": WARNING: %s had a default value. This means that "
                      "token tracking was off when it was created. If this "
                      "is a unit test, make sure to create your "
                      "SlateBuilder before your test objects.\n",
                      name_phrase.c_str());
        }
        else
        {
            ++valid_id_count;
        }

        return is_known_id;
    }

    /**
     * Return the first invalid ID given out since the last enable().
     *
     * @return The first ID.
     */
    slate_element_t SlateTokenAccountant::get_first_invalid_id() const
    {
        return slate_id_build_invalid(first_invalid_count);
    }

    /**
     * Return one past the the last invalid ID given out since the last
     * enable().
     *
     * @return One past the last ID.
     */
    slate_element_t SlateTokenAccountant::get_last_invalid_id() const
    {
        return slate_id_build_invalid(next_invalid_count);
    }

    /**
     * Enable the accountant.
     */
    void SlateTokenAccountant::enable()
    {
        /*
         * Slate token accountant should only be re-enabled if auto_re_enable is
         * set to true, which is only the case for unit-tests. If this assert
         * fires, a new SlateBuilder is being created after a previous one has
         * already been built. Multiple root SlateBuilders is not correctly
         * supported in a vehicle and it shouldn't be relied upon.
         */
        SacAssert(_slate_token_accountant_enabled ||
                  _slate_token_accountant_auto_re_enable);

        if (_slate_token_accountant_auto_re_enable)
        {
            _slate_token_accountant_enabled = true;
        }
    }

    /**
     * Check for any uninitialized tokens.
     *
     * @return True if all tokens were deleted or initialized.
     */
    bool SlateTokenAccountant::check_for_uninitialized_tokens() const
    {
        /*
         * Ensure no default tokens were created and not destroyed. This
         * could indicate that static initialization is not working the
         * way we expected and statically-created tokens weren't being
         * tracked.
         *
         * If this is a unit test, then it is likely you created a slate
         * token in between building one slate and creating another. For
         * example:
         *
         * ---
         * SlateBuilder builder;
         * builder.build();
         *
         * WriteToken<int> tok;
         *
         * SlateBuilder builder2;
         * builder2.build();
         * ---
         *
         * Instead, you must do this:
         *
         * ---
         * SlateBuilder builder;
         * builder.build();
         *
         * SlateBuilder builder2;
         *
         * WriteToken<int> tok;
         *
         * builder2.build();
         * ---
         *
         */
        if (SacIfNeq(_slate_default_token_count, 0))
        {
            SacPrefix();
            dbnprintf(200,
                      ": ERROR: %zu leaked token(s) detected!\n",
                      _slate_default_token_count);
            return false;
        }

        /*
         * Check that all tokens are accounted for.
         */
        SacAbortIfNeq(valid_id_count + id_frames.size(),
                      next_invalid_count - first_invalid_count,
                      false);

        /*
         * Print out any uninitialized tokens left.
         */
        if (!id_frames.empty())
        {
            dbstring("----------------------------------------\n");
            SacPrefix();
            dbstring(": ERROR: Uninitialized token(s) detected!:\n");
            dbstring("----------------------------------------\n");

            for (auto it = id_frames.begin(); it != id_frames.end(); ++it)
            {
                SacPrefix();
                dbnprintf(200, ": id: 0x%016llx.\n", it->first);

                if (!it->second.sac_context.empty())
                {
                    const std::string msg = ": Surrounding Sac macro at " +
                                            it->second.sac_context + "\n";
                    SacPrefix();
                    dbstring(msg.c_str());
                }
                dbstring("----------------------------------------\n");
            }

            return false;
        }

        return true;
    }

    /**
     * Acknowledge all outstanding IDs. Clears all valid ID entries,
     * moves the first ID to the current value, and disables the
     * accountant.
     *
     * @return True on success.
     */
    bool SlateTokenAccountant::acknowledge_and_disable()
    {
        _slate_token_accountant_enabled = false;
        first_invalid_count = next_invalid_count;
        id_frames.clear();
        valid_id_count = 0;

        return true;
    }

} /* end namespace Drone */