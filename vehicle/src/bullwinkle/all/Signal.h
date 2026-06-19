/**
 * @author Richard Bao
 * @date   01/28/2025
 */

#ifndef SIGNAL_H
#define SIGNAL_H

#include "src/bullwinkle/all/core/sac.h"
#include "src/bullwinkle/all/core/drone_types.h"

#include <functional>
#include <list>
#include <memory>
#include <type_traits>

namespace Drone
{
    /*
     * USAGE NOTES FOR THE SIGNAL LIBRARY
     *
     * This library implements a mechanism for communicating via function calls
     * between disparate parts of a code base. The core of this library is the
     * Signal class, which represents a set of functions that can be called.
     *
     * The Signal, Slot, SignalConnection, and SignalHandler classes are meant
     * to be used externally. The other classes in this file are used internally
     * by the library.
     *
     * Below is an example usage:
     *
     * class Producer
     * {
     * public:
     *     Producer():
     *         count(0)
     *     {}
     *
     *     dispatch()
     *     {
     *         sig.emit(count);
     *         count++;
     *     }
     *
     *     Signal<void, int> sig;
     *
     * private:
     *     int count;
     * };
     *
     * class Consumer: public SignalHandler
     * {
     * public:
     *     Consumer()
     *     {}
     *
     *     bool init(Signal<void, int> &sig)
     *     {
     *         SacAbortIfNot(sig.connect(make_slot(*this, &Consumer::callback)),
     *                       false);
     *         return true;
     *     }
     *
     * private:
     *     void callback(int x)
     *     {
     *         dbnprintf(100, "%d\n", x);
     *     }
     * };
     *
     * int main()
     * {
     *     Producer producer;
     *
     *     Consumer consumer1;
     *     SacAbortIfNot(consumer1.init(producer.sig), false);
     *
     *     Consumer consumer2;
     *     SacAbortIfNot(consumer2.init(producer.sig), false);
     * }
     */

    /**
     * A mechanism that a SignalHandler needs to invalidate a function held by
     * associated Slots.
     */
    class FunctionInvalidator
    {
    public:
        /**
         * Constructor.
         */
        FunctionInvalidator()
        {}

        /**
         * Destructor.
         */
        virtual ~FunctionInvalidator()
        {}

        /**
         * Invalidate the function held by the associated Slots.
         */
        virtual void invalidate() = 0;

        /**
         * Determine if the function held by the Slots has expired (i.e. all
         * Slots that held it have gone out of scope).
         */
        virtual bool expired() const = 0;

    private:
        /*
         * This object is only ever meant to be created inside a smart pointer,
         * so there is no reason to allow copying it.
         */
        SX_DISALLOW_COPY_AND_ASSIGN(FunctionInvalidator);
    };

    /**
     * A mechanism that a SignalHandler needs to invalidate a function held by
     * associated Slots.
     *
     * This is a concrete implementation of FunctionInvalidator that knows about
     * the signature of the function and holds a weak pointer to it.
     */
    template <class R, class... Args>
    class FunctionWeakPointer: public FunctionInvalidator
    {
    public:
        /**
         * Constructor.
         */
        FunctionWeakPointer(): fwptr()
        {}

        /**
         * Constructor.
         *
         * @param fsptr A shared_ptr to the function.
         */
        explicit FunctionWeakPointer(
            const std::shared_ptr<std::function<R(Args...)>> &fsptr):
            fwptr(fsptr)
        {}

        /**
         * See FunctionInvalidator.
         */
        void invalidate() override
        {
            const std::shared_ptr<std::function<R(Args...)>> fsptr =
                fwptr.lock();

            if (fsptr)
            {
                (*fsptr) = std::function<R(Args...)>();
            }
        }

        /**
         * See FunctionInvalidator.
         */
        bool expired() const override
        {
            return fwptr.expired();
        }

    private:
        /**
         * A weak pointer to a function.
         */
        std::weak_ptr<std::function<R(Args...)>> fwptr;
    };

    /**
     * Any class that wants one of its member functions to be called by a Slot
     * must inherit from SignalHandler. SignalHandler takes care of notifying
     * the Slot when the SignalHandler is destroyed.
     */
    class SignalHandler
    {
    public:
        /**
         * Constructor.
         */
        SignalHandler(): function_invalidators()
        {}

        /**
         * Destructor.
         */
        virtual ~SignalHandler()
        {
            for (const auto &fi : function_invalidators)
            {
                SacDebugAssert(fi);

                if (fi)
                {
                    fi->invalidate();
                }
            }
        }

        /**
         * Register a FunctionInvalidator with this SignalHandler so that the
         * invalidator can be invoked when the handler goes out of scope.
         *
         * @param fi A unique_ptr to the FunctionInvalidator.
         */
        void
        register_function_invalidator(std::unique_ptr<FunctionInvalidator> &&fi)
        {
            /*
             * Add the new invalidator.
             */
            if (fi)
            {
                function_invalidators.emplace_back(std::move(fi));
            }

            /*
             * Clear out any old invalidators that have since expired.
             */
            const auto expired =
                [](const std::unique_ptr<FunctionInvalidator> &list_fi) {
                    return !list_fi || list_fi->expired();
                };

            function_invalidators.remove_if(expired);
        }

    private:
        /*
         * Note that it is important that SignalHandler should not allow copy
         * construction, copy assignment, move construction, or move assignment.
         * This is because the contents of function_invalidators must be unique
         * to this object and never associated with another object.
         */
        SX_DISALLOW_COPY_AND_ASSIGN(SignalHandler);

        /**
         * The list of function invalidators.
         */
        std::list<std::unique_ptr<FunctionInvalidator>> function_invalidators;
    };

    /**
     * Forward declare Signal since it is used in Slot.
     */
    template <class R, class... Args>
    class Signal;

    /**
     * A wrapper around a function that works safely even if the object that the
     * underlying function is associated with goes out of scope.
     *
     * @tparam R The return type of the connected functions.
     * @tparam Args Variadic list of parameters for the connected functions.
     */
    template <class R, class... Args>
    class Slot
    {
    public:
        /**
         * Constructor.
         */
        Slot(): fsptr(), attached_handler(nullptr)
        {}

        /**
         * Construct a Slot for the given function.
         *
         * @note This is for functions which are not members of a class.
         *
         * @param non_member_f Pointer to a non-member function.
         */
        explicit Slot(R (*non_member_f)(Args...)):
            fsptr(),
            attached_handler(nullptr)
        {
            SacAssert(init_non_member(non_member_f));
        }

        /**
         * Construct a Slot for the given function.
         *
         * @note This is for functions which are members of a class.
         *
         * @tparam HandlerType An object that inherits from SignalHandler.
         * @tparam MemberType The type associated with the member function.
         *
         * @param handler Reference to the object the given function is part of.
         *                Note that this must be of type SignalHandler.
         * @param member_f Pointer to a member function of the object.
         */
        template <class HandlerType, class MemberType>
        Slot(HandlerType &handler, R (MemberType::*member_f)(Args...)):
            fsptr(),
            attached_handler(nullptr)
        {
            SacAssert(init_member(handler, member_f));
        }

        /**
         * Same as above but for const method.
         */
        template <class HandlerType, class MemberType>
        Slot(HandlerType &handler, R (MemberType::*member_f)(Args...) const):
            fsptr(),
            attached_handler(nullptr)
        {
            SacAssert(init_member(handler, member_f));
        }

        /**
         * Initialize this Slot from another one by copying its function,
         * binding arguments to that function, and registering with its
         * SignalHandler, if necessary.
         *
         * @note This is not a constructor to prevent it from getting confused
         *       with a copy constructor if the list of arguments is the same.
         *
         * @tparam BindArgs Variadic list of parameters to bind to the function
         *                  when it is called. May be placeholders, e.g.
         *                  "std::placeholders::_1".
         * @tparam InArgs Variadic list of parameters for the given function.
         *
         * @param slot The slot that this Slot should be initialized from.
         * @param bindargs The arguments to be bound to the function.
         *
         * @return True on success.
         */
        template <class... BindArgs, class... InArgs>
        bool
        init_from_slot(const Slot<R, InArgs...> &slot, BindArgs... bindargs)
        {
            SacAbortIf(fsptr, false);
            SacAbortIf(attached_handler, false);

            SacAbortIfNot(slot, false);
            SacAbortIfNot(slot.fsptr, false);

            const std::function<R(InArgs...)> old_f = *slot.fsptr;
            SacAbortIfNot(old_f, false);

            const std::function<R(Args...)> new_f =
                std::bind(old_f, bindargs...);
            const std::shared_ptr<std::function<R(Args...)>> new_fsptr(
                new std::function<R(Args...)>(new_f));

            SacAbortIfNot(new_fsptr, false);
            SacAbortIfNot((*new_fsptr), false);

            if (slot.attached_handler)
            {
                std::unique_ptr<FunctionInvalidator> fi(
                    new FunctionWeakPointer<R, Args...>(new_fsptr));
                SacAbortIfNot(fi, false);

                attached_handler = slot.attached_handler;
                SacAbortIfNot(attached_handler, false);

                attached_handler->register_function_invalidator(std::move(fi));
            }

            /*
             * Setting the function should be the last action performed since
             * its validity stands in for the validity of the Slot.
             */
            fsptr = new_fsptr;
            SacAbortIfNot(fsptr, false);
            SacAbortIfNot((*fsptr), false);

            return true;
        }

        /**
         * Operator bool. Determines if the Slot is valid (i.e. can be safely
         * executed).
         *
         * @return True if the Slot is valid.
         */
        explicit operator bool() const
        {
            if (!fsptr)
            {
                return false;
            }

            return fsptr->operator bool();
        }

        /**
         * Function call operator. This will call the underlying function with
         * the given arguments in a safe way (i.e. if the Slot is invalid then
         * a default value will be returned.
         *
         * @param args The arguments to give to the underlying function.
         *
         * @return The return value of the underlying function.
         */
        R operator()(Args... args) const
        {
            if (!fsptr)
            {
                constexpr bool ptr_is_valid = false;
                SacAssert(ptr_is_valid);
                return R();
            }

            const std::function<R(Args...)> &f = *fsptr;

            if (!f)
            {
                constexpr bool function_is_valid = false;
                SacAssert(function_is_valid);
                return R();
            }

            return f(args...);
        }

    private:
        /*
         * Allow Slots of other types to access each others' internal variables.
         */
        template <class R_Other, class... Args_Other>
        friend class Slot;

        /*
         * Allow Signal to access the pointer to the function.
         */
        friend class Signal<R, Args...>;

        /**
         * Initialize this Slot from a non-member function.
         *
         * @param non_member_f Pointer to a non-member function.
         *
         * @return True on success.
         */
        bool init_non_member(R (*non_member_f)(Args...))
        {
            SacAbortIf(fsptr, false);
            SacAbortIf(attached_handler, false);

            SacAbortIfNot(non_member_f, false);

            /*
             * Setting the function should be the last action performed since
             * its validity stands in for the validity of the Slot.
             */
            fsptr = std::shared_ptr<std::function<R(Args...)>>(
                new std::function<R(Args...)>(non_member_f));
            SacAbortIfNot(fsptr, false);
            SacAbortIfNot((*fsptr), false);

            return true;
        }

        /**
         * Initialize this Slot from a member function.
         *
         * @tparam HandlerType An object that inherits from SignalHandler.
         * @tparam MemberType The type associated with the member function.
         *
         * @param handler Reference to the object the given function is part of.
         *                Note that this must be of type SignalHandler.
         * @param member_f Pointer to a member function of the object.
         *
         * @return True on success.
         */
        template <class HandlerType, class MemberType>
        bool
        init_member(HandlerType &handler, R (MemberType::*member_f)(Args...))
        {
            SacAbortIfNot(member_f, false);

            static_assert(std::is_base_of<MemberType, HandlerType>::value, "");

            HandlerType *ptr = &handler;
            const std::function<R(Args...)> new_f =
                [ptr, member_f](Args... args) -> R {
                return ((ptr)->*(member_f))(args...);
            };

            SacAbortIfNot(init_member_common(handler, new_f), false);

            return true;
        }

        /**
         * Same as above but for const method.
         */
        template <class HandlerType, class MemberType>
        bool init_member(HandlerType &handler,
                         R (MemberType::*member_f)(Args...) const)
        {
            SacAbortIfNot(member_f, false);

            static_assert(std::is_base_of<MemberType, HandlerType>::value, "");

            HandlerType *ptr = &handler;
            const std::function<R(Args...)> new_f =
                [ptr, member_f](Args... args) -> R {
                return ((ptr)->*(member_f))(args...);
            };

            SacAbortIfNot(init_member_common(handler, new_f), false);

            return true;
        }

        /**
         * Initialize this Slot from a member function.
         *
         * @tparam HandlerType An object that inherits from SignalHandler.
         *
         * @param handler Reference to the object the given function is part of.
         *                Note that this must be of type SignalHandler.
         * @param new_f The new function.
         *
         * @return True on success.
         */
        template <class HandlerType>
        bool init_member_common(HandlerType &handler,
                                const std::function<R(Args...)> &new_f)
        {
            SacAbortIf(fsptr, false);
            SacAbortIf(attached_handler, false);

            SacAbortIfNot(new_f, false);

            const std::shared_ptr<std::function<R(Args...)>> new_fsptr(
                new std::function<R(Args...)>(new_f));

            SacAbortIfNot(new_fsptr, false);
            SacAbortIfNot((*new_fsptr), false);

            static_assert(std::is_base_of<SignalHandler, HandlerType>::value,
                          "A Slot can only be used with a member function of a "
                          "class that inherits from SignalHandler.");
            attached_handler = static_cast<SignalHandler *>(&handler);
            SacAbortIfNot(attached_handler, false);

            std::unique_ptr<FunctionInvalidator> fi(
                new FunctionWeakPointer<R, Args...>(new_fsptr));
            SacAbortIfNot(fi, false);

            attached_handler->register_function_invalidator(std::move(fi));

            /*
             * Setting the function should be the last action performed since
             * its validity stands in for the validity of the Slot.
             */
            fsptr = new_fsptr;
            SacAbortIfNot(fsptr, false);
            SacAbortIfNot((*fsptr), false);

            return true;
        }

        /**
         * The function associated with this Slot.
         */
        std::shared_ptr<std::function<R(Args...)>> fsptr;

        /**
         * The SignalHandler that the function is attached to if it is a member
         * function (this will be nullptr for non-member functions).
         */
        SignalHandler *attached_handler;
    };

    /**
     * A helper to create a Slot from a function.
     *
     * @note This is for functions which are not members of a class.
     *
     * @tparam R The return type of the connected functions.
     * @tparam Args Variadic list of parameters for the connected functions.
     *
     * @param non_member_f Pointer to a non-member function.
     *
     * @return The created Slot.
     */
    template <class R, class... Args>
    Slot<R, Args...> make_slot(R (*non_member_f)(Args...))
    {
        return Slot<R, Args...>(non_member_f);
    }

    /**
     * A helper to create a Slot from a function.
     *
     * @note This is for functions which are members of a class.
     *
     * @tparam HandlerType An object that inherits from SignalHandler.
     * @tparam MemberType The type associated with the member function.
     * @tparam R The return type of the connected functions.
     * @tparam Args Variadic list of parameters for the connected functions.
     *
     * @param handler Reference to the object the given function is part of.
     *                Note that this must be of type SignalHandler.
     * @param member_f Pointer to a member function of the object.
     *
     * @return The created Slot.
     */
    template <class HandlerType, class MemberType, class R, class... Args>
    Slot<R, Args...>
    make_slot(HandlerType &handler, R (MemberType::*member_f)(Args...))
    {
        return Slot<R, Args...>(handler, member_f);
    }

    /**
     * Same as above but for const method.
     */
    template <class HandlerType, class MemberType, class R, class... Args>
    Slot<R, Args...>
    make_slot(HandlerType &handler, R (MemberType::*member_f)(Args...) const)
    {
        return Slot<R, Args...>(handler, member_f);
    }

    /**
     * A helper to create a Slot from another Slot. The new Slot binds the given
     * value to the last argument of the given Slot.
     *
     * @note This is for a new Slot that takes 0 parameters.
     *
     * @tparam BindArg The type of the argument to be bound to the given Slot.
     * @tparam BindArgIn The type of the bind argument passed in (this may be
     *                   different to allow implicit conversion).
     * @tparam R The return type of the connected functions.
     *
     * @param slot The given Slot.
     * @param bindarg The given argument to bind.
     *
     * @return The created Slot.
     */
    template <class BindArg, class BindArgIn, class R>
    Slot<R> slot_bind(const Slot<R, BindArg> &slot, const BindArgIn &bindarg)
    {
        static_assert(!std::is_reference<BindArg>::value, "");

        Slot<R> empty_slot;
        Slot<R> new_slot;
        SacAbortIfNot(new_slot.init_from_slot(slot, bindarg), empty_slot);
        return new_slot;
    }

    /**
     * See slot_bind() for new Slots with 0 function arguments for more details.
     *
     * @note This is for a new Slot that takes 1 parameter.
     *
     * @tparam Arg1 Function argument 1.
     */
    template <class BindArg, class BindArgIn, class R, class Arg1>
    Slot<R, Arg1>
    slot_bind(const Slot<R, Arg1, BindArg> &slot, const BindArgIn &bindarg)
    {
        static_assert(!std::is_reference<BindArg>::value, "");

        Slot<R, Arg1> empty_slot;
        Slot<R, Arg1> new_slot;
        SacAbortIfNot(
            new_slot.init_from_slot(slot, std::placeholders::_1, bindarg),
            empty_slot);
        return new_slot;
    }

    /**
     * See slot_bind() for new Slots with 0 function arguments for more details.
     *
     * @note This is for a new Slot that takes 2 parameters.
     *
     * @tparam Arg1 Function argument 1.
     * @tparam Arg2 Function argument 2.
     */
    template <class BindArg, class BindArgIn, class R, class Arg1, class Arg2>
    Slot<R, Arg1, Arg2> slot_bind(const Slot<R, Arg1, Arg2, BindArg> &slot,
                                  const BindArgIn &bindarg)
    {
        static_assert(!std::is_reference<BindArg>::value, "");

        Slot<R, Arg1, Arg2> empty_slot;
        Slot<R, Arg1, Arg2> new_slot;
        SacAbortIfNot(new_slot.init_from_slot(slot,
                                              std::placeholders::_1,
                                              std::placeholders::_2,
                                              bindarg),
                      empty_slot);
        return new_slot;
    }

    /**
     * See slot_bind() for new Slots with 0 function arguments for more details.
     *
     * @note This is for a new Slot that takes 3 parameters.
     *
     * @tparam Arg1 Function argument 1.
     * @tparam Arg2 Function argument 2.
     * @tparam Arg3 Function argument 3.
     */
    template <class BindArg,
              class BindArgIn,
              class R,
              class Arg1,
              class Arg2,
              class Arg3>
    Slot<R, Arg1, Arg2, Arg3>
    slot_bind(const Slot<R, Arg1, Arg2, Arg3, BindArg> &slot,
              const BindArgIn &bindarg)
    {
        static_assert(!std::is_reference<BindArg>::value, "");

        Slot<R, Arg1, Arg2, Arg3> empty_slot;
        Slot<R, Arg1, Arg2, Arg3> new_slot;
        SacAbortIfNot(new_slot.init_from_slot(slot,
                                              std::placeholders::_1,
                                              std::placeholders::_2,
                                              std::placeholders::_3,
                                              bindarg),
                      empty_slot);
        return new_slot;
    }

    /**
     * See slot_bind() for new Slots with 0 function arguments for more details.
     *
     * @note This is for a new Slot that takes 4 parameters.
     *
     * @tparam Arg1 Function argument 1.
     * @tparam Arg2 Function argument 2.
     * @tparam Arg3 Function argument 3.
     * @tparam Arg4 Function argument 4.
     */
    template <class BindArg,
              class BindArgIn,
              class R,
              class Arg1,
              class Arg2,
              class Arg3,
              class Arg4>
    Slot<R, Arg1, Arg2, Arg3, Arg4>
    slot_bind(const Slot<R, Arg1, Arg2, Arg3, Arg4, BindArg> &slot,
              const BindArgIn &bindarg)
    {
        static_assert(!std::is_reference<BindArg>::value, "");

        Slot<R, Arg1, Arg2, Arg3, Arg4> empty_slot;
        Slot<R, Arg1, Arg2, Arg3, Arg4> new_slot;
        SacAbortIfNot(new_slot.init_from_slot(slot,
                                              std::placeholders::_1,
                                              std::placeholders::_2,
                                              std::placeholders::_3,
                                              std::placeholders::_4,
                                              bindarg),
                      empty_slot);
        return new_slot;
    }

    /**
     * See slot_bind() for new Slots with 0 function arguments for more details.
     *
     * @note This is for a new Slot that takes 0 parameters.
     * @note This version takes 2 bind arguments.
     *
     * @tparam BindArg1 The type of the first argument to be bound to the
     *                  given Slot.
     * @tparam BindArgIn1 The type of the first bind argument passed in (this
     *                    may be different to allow implicit conversion).
     * @tparam BindArg2 The type of the second argument to be bound to the
     *                  given Slot.
     * @tparam BindArgIn2 The type of the second bind argument passed in (this
     *                    may be different to allow implicit conversion).
     *
     * @param bindarg1 The first given argument to bind.
     * @param bindarg2 The second given argument to bind.
     */
    template <class BindArg1,
              class BindArgIn1,
              class BindArg2,
              class BindArgIn2,
              class R>
    Slot<R> slot_bind(const Slot<R, BindArg1, BindArg2> &slot,
                      const BindArgIn1 &bindarg1,
                      const BindArgIn2 &bindarg2)
    {
        static_assert(!std::is_reference<BindArg1>::value, "");
        static_assert(!std::is_reference<BindArg2>::value, "");

        Slot<R> empty_slot;
        Slot<R> new_slot;
        SacAbortIfNot(new_slot.init_from_slot(slot, bindarg1, bindarg2),
                      empty_slot);
        return new_slot;
    }

    /**
     * See slot_bind() for new Slots with 0 function arguments and 2 bind
     * arguments for more details.
     *
     * @note This is for a new Slot that takes 1 parameter.
     * @note This version takes 2 bind arguments.
     *
     * @tparam Arg1 Function argument 1.
     */
    template <class BindArg1,
              class BindArgIn1,
              class BindArg2,
              class BindArgIn2,
              class R,
              class Arg1>
    Slot<R, Arg1> slot_bind(const Slot<R, Arg1, BindArg1, BindArg2> &slot,
                            const BindArgIn1 &bindarg1,
                            const BindArgIn2 &bindarg2)
    {
        static_assert(!std::is_reference<BindArg1>::value, "");
        static_assert(!std::is_reference<BindArg2>::value, "");

        Slot<R, Arg1> empty_slot;
        Slot<R, Arg1> new_slot;
        SacAbortIfNot(new_slot.init_from_slot(
                          slot, std::placeholders::_1, bindarg1, bindarg2),
                      empty_slot);
        return new_slot;
    }

    /**
     * See slot_bind() for new Slots with 0 function arguments and 2 bind
     * arguments for more details.
     *
     * @note This is for a new Slot that takes 2 parameters.
     * @note This version takes 2 bind arguments.
     *
     * @tparam Arg1 Function argument 1.
     * @tparam Arg2 Function argument 2.
     */
    template <class BindArg1,
              class BindArgIn1,
              class BindArg2,
              class BindArgIn2,
              class R,
              class Arg1,
              class Arg2>
    Slot<R, Arg1, Arg2>
    slot_bind(const Slot<R, Arg1, Arg2, BindArg1, BindArg2> &slot,
              const BindArgIn1 &bindarg1,
              const BindArgIn2 &bindarg2)
    {
        static_assert(!std::is_reference<BindArg1>::value, "");
        static_assert(!std::is_reference<BindArg2>::value, "");

        Slot<R, Arg1, Arg2> empty_slot;
        Slot<R, Arg1, Arg2> new_slot;
        SacAbortIfNot(new_slot.init_from_slot(slot,
                                              std::placeholders::_1,
                                              std::placeholders::_2,
                                              bindarg1,
                                              bindarg2),
                      empty_slot);
        return new_slot;
    }

    /**
     * See slot_bind() for new Slots with 0 function arguments for more details.
     *
     * @note This is for a new Slot that takes 0 parameters.
     * @note This version takes 3 bind arguments.
     *
     * @tparam BindArg1 The type of the first argument to be bound to the
     *                  given Slot.
     * @tparam BindArgIn1 The type of the first bind argument passed in (this
     *                    may be different to allow implicit conversion).
     * @tparam BindArg2 The type of the second argument to be bound to the
     *                  given Slot.
     * @tparam BindArgIn2 The type of the second bind argument passed in (this
     *                    may be different to allow implicit conversion).
     * @tparam BindArg3 The type of the third argument to be bound to the
     *                  given Slot.
     * @tparam BindArgIn3 The type of the third bind argument passed in (this
     *                    may be different to allow implicit conversion).
     *
     * @param bindarg1 The first given argument to bind.
     * @param bindarg2 The second given argument to bind.
     * @param bindarg3 The third given argument to bind.
     */
    template <class BindArg1,
              class BindArgIn1,
              class BindArg2,
              class BindArgIn2,
              class BindArg3,
              class BindArgIn3,
              class R>
    Slot<R> slot_bind(const Slot<R, BindArg1, BindArg2, BindArg3> &slot,
                      const BindArgIn1 &bindarg1,
                      const BindArgIn2 &bindarg2,
                      const BindArgIn3 &bindarg3)
    {
        static_assert(!std::is_reference<BindArg1>::value, "");
        static_assert(!std::is_reference<BindArg2>::value, "");
        static_assert(!std::is_reference<BindArg3>::value, "");

        Slot<R> empty_slot;
        Slot<R> new_slot;
        SacAbortIfNot(
            new_slot.init_from_slot(slot, bindarg1, bindarg2, bindarg3),
            empty_slot);
        return new_slot;
    }

    /**
     * See slot_bind() for new Slots with 0 function arguments and 3 bind
     * arguments for more details.
     *
     * @note This is for a new Slot that takes 1 parameter.
     * @note This version takes 3 bind arguments.
     *
     * @tparam Arg1 Function argument 1.
     */
    template <class BindArg1,
              class BindArgIn1,
              class BindArg2,
              class BindArgIn2,
              class BindArg3,
              class BindArgIn3,
              class R,
              class Arg1>
    Slot<R, Arg1>
    slot_bind(const Slot<R, Arg1, BindArg1, BindArg2, BindArg3> &slot,
              const BindArgIn1 &bindarg1,
              const BindArgIn2 &bindarg2,
              const BindArgIn3 &bindarg3)
    {
        static_assert(!std::is_reference<BindArg1>::value, "");
        static_assert(!std::is_reference<BindArg2>::value, "");
        static_assert(!std::is_reference<BindArg3>::value, "");

        Slot<R, Arg1> empty_slot;
        Slot<R, Arg1> new_slot;
        SacAbortIfNot(new_slot.init_from_slot(slot,
                                              std::placeholders::_1,
                                              bindarg1,
                                              bindarg2,
                                              bindarg3),
                      empty_slot);
        return new_slot;
    }

    /**
     * See slot_bind() for new Slots with 0 function arguments and 3 bind
     * arguments for more details.
     *
     * @note This is for a new Slot that takes 2 parameters.
     * @note This version takes 3 bind arguments.
     *
     * @tparam Arg1 Function argument 1.
     * @tparam Arg2 Function argument 2.
     */
    template <class BindArg1,
              class BindArgIn1,
              class BindArg2,
              class BindArgIn2,
              class BindArg3,
              class BindArgIn3,
              class R,
              class Arg1,
              class Arg2>
    Slot<R, Arg1, Arg2>
    slot_bind(const Slot<R, Arg1, Arg2, BindArg1, BindArg2, BindArg3> &slot,
              const BindArgIn1 &bindarg1,
              const BindArgIn2 &bindarg2,
              const BindArgIn3 &bindarg3)
    {
        static_assert(!std::is_reference<BindArg1>::value, "");
        static_assert(!std::is_reference<BindArg2>::value, "");
        static_assert(!std::is_reference<BindArg3>::value, "");

        Slot<R, Arg1, Arg2> empty_slot;
        Slot<R, Arg1, Arg2> new_slot;
        SacAbortIfNot(new_slot.init_from_slot(slot,
                                              std::placeholders::_1,
                                              std::placeholders::_2,
                                              bindarg1,
                                              bindarg2,
                                              bindarg3),
                      empty_slot);
        return new_slot;
    }

    /**
     * See slot_bind() for new Slots with 0 function arguments and 3 bind
     * arguments for more details.
     *
     * @note This is for a new Slot that takes 3 parameters.
     * @note This version takes 3 bind arguments.
     *
     * @tparam Arg1 Function argument 1.
     * @tparam Arg2 Function argument 2.
     * @tparam Arg3 Function argument 3.
     */
    template <class BindArg1,
              class BindArgIn1,
              class BindArg2,
              class BindArgIn2,
              class BindArg3,
              class BindArgIn3,
              class R,
              class Arg1,
              class Arg2,
              class Arg3>
    Slot<R, Arg1, Arg2, Arg3>
    slot_bind(const Slot<R, Arg1, Arg2, BindArg1, BindArg2, BindArg3> &slot,
              const BindArgIn1 &bindarg1,
              const BindArgIn2 &bindarg2,
              const BindArgIn3 &bindarg3)
    {
        static_assert(!std::is_reference<BindArg1>::value, "");
        static_assert(!std::is_reference<BindArg2>::value, "");
        static_assert(!std::is_reference<BindArg3>::value, "");

        Slot<R, Arg1, Arg2, Arg3> empty_slot;
        Slot<R, Arg1, Arg2, Arg3> new_slot;
        SacAbortIfNot(new_slot.init_from_slot(slot,
                                              std::placeholders::_1,
                                              std::placeholders::_2,
                                              std::placeholders::_3,
                                              bindarg1,
                                              bindarg2,
                                              bindarg3),
                      empty_slot);
        return new_slot;
    }

    /**
     * A helper to create a Slot from another Slot. The new Slot hides the last
     * argument of the new Slot when calling the function of the given Slot.
     *
     * @note This is for a given Slot that takes 0 parameters.
     *
     * @tparam HideArg The type of the argument to be hidden from the given
     *                 Slot.
     * @tparam R The return type of the connected functions.
     *
     * @param slot The given Slot.
     *
     * @return The created Slot.
     */
    template <class HideArg, class R>
    Slot<R, HideArg> slot_hide(const Slot<R> &slot)
    {
        Slot<R, HideArg> empty_slot;
        Slot<R, HideArg> new_slot;
        SacAbortIfNot(new_slot.init_from_slot(slot), empty_slot);
        return new_slot;
    }

    /**
     * See slot_hide() for given Slots with 0 function arguments for more
     * details.
     *
     * @note This is for a given Slot that takes 1 parameter.
     *
     * @tparam Arg1 Function argument 1.
     */
    template <class HideArg, class R, class Arg1>
    Slot<R, Arg1, HideArg> slot_hide(const Slot<R, Arg1> &slot)
    {
        Slot<R, Arg1, HideArg> empty_slot;
        Slot<R, Arg1, HideArg> new_slot;
        SacAbortIfNot(new_slot.init_from_slot(slot, std::placeholders::_1),
                      empty_slot);
        return new_slot;
    }

    /**
     * See slot_hide() for given Slots with 0 function arguments for more
     * details.
     *
     * @note This is for a given Slot that takes 2 parameters.
     *
     * @tparam Arg1 Function argument 1.
     * @tparam Arg2 Function argument 2.
     */
    template <class HideArg, class R, class Arg1, class Arg2>
    Slot<R, Arg1, Arg2, HideArg> slot_hide(const Slot<R, Arg1, Arg2> &slot)
    {
        Slot<R, Arg1, Arg2, HideArg> empty_slot;
        Slot<R, Arg1, Arg2, HideArg> new_slot;
        SacAbortIfNot(new_slot.init_from_slot(slot,
                                              std::placeholders::_1,
                                              std::placeholders::_2),
                      empty_slot);
        return new_slot;
    }

    /**
     * See slot_hide() for given Slots with 0 function arguments for more
     * details.
     *
     * @note This is for a given Slot that takes 3 parameters.
     *
     * @tparam Arg1 Function argument 1.
     * @tparam Arg2 Function argument 2.
     * @tparam Arg3 Function argument 3.
     */
    template <class HideArg, class R, class Arg1, class Arg2, class Arg3>
    Slot<R, Arg1, Arg2, Arg3, HideArg>
    slot_hide(const Slot<R, Arg1, Arg2, Arg3> &slot)
    {
        Slot<R, Arg1, Arg2, Arg3, HideArg> empty_slot;
        Slot<R, Arg1, Arg2, Arg3, HideArg> new_slot;
        SacAbortIfNot(new_slot.init_from_slot(slot,
                                              std::placeholders::_1,
                                              std::placeholders::_2,
                                              std::placeholders::_3),
                      empty_slot);
        return new_slot;
    }

    /*
     * NOTE: The following ReturnAccumulator classes correspond to the AccumType
     * duck type used by Signal::emit(). This duck type must be equivalent to
     * the following form:
     *
     *     template <class R, class... Args>
     *     class CLASS_NAME
     *     {
     *     public:
     *         void accumulate(const std::function<R(Args...)> &f, Args... args)
     *         {}
     *
     *         const R &get_accumulated() const
     *         {}
     *     };
     *
     * The accumulate() method shall call the given function with the given args
     * and then may optionally accumulate its return value into some internal
     * state. The get_accumulated() method shall return the accumulated value.
     */

    /**
     * An AccumType that stores the return value of the function into a single
     * value.
     *
     * @tparam R The return type of the function.
     * @tparam Args Variadic list of parameters for the function.
     *
     * Note: This is not applicable to functions with void return types.
     */
    template <class R, class... Args>
    class ReturnAccumulator
    {
    public:
        /**
         * Constructor.
         */
        ReturnAccumulator(): r()
        {}

        /**
         * Call the given function with the given arguments and store its return
         * value.
         *
         * @param f The function to call.
         * @param args The arguments to supply to the function.
         */
        inline void accumulate(const std::function<R(Args...)> &f, Args... args)
        {
            r = f(args...);
        }

        /**
         * Get the saved return value from the last call to accumulate().
         *
         * @return The saved return value.
         */
        inline const R &get_accumulated() const
        {
            return r;
        }

    private:
        /**
         * State for the saved return value.
         */
        R r;
    };

    /**
     * An AccumType that can be used with functions that have a void return
     * type.
     *
     * @tparam Args Variadic list of parameters for the function.
     */
    template <class... Args>
    class ReturnAccumulator<void, Args...>
    {
    public:
        /**
         * Call the given function with the given arguments.
         *
         * @param f The function to call.
         * @param args The arguments to supply to the function.
         */
        inline void
        accumulate(const std::function<void(Args...)> &f, Args... args)
        {
            f(args...);
        }

        /**
         * Get the accumulated value (for void this does nothing).
         */
        inline void get_accumulated() const
        {
            return;
        }
    };

    /**
     * An object returned by Signal::connect() that allows the connected Slot to
     * be disconnected from the Signal.
     */
    class SignalConnection
    {
    public:
        /**
         * Constructor.
         */
        SignalConnection(): disconnect_slot(), is_valid_slot()
        {}

        /**
         * Constructor.
         *
         * @param _disconnect_slot Callback to the Signal to disconnect the
         *                         Slot.
         * @param _is_valid_slot Callback to the Signal to check if the Slot is
         *                       valid.
         */
        SignalConnection(const Slot<void> &_disconnect_slot,
                         const Slot<bool> &_is_valid_slot):
            disconnect_slot(_disconnect_slot),
            is_valid_slot(_is_valid_slot)
        {}

        /**
         * Disconnect the associated function from its Slot.
         */
        void disconnect()
        {
            if (!disconnect_slot)
            {
                return;
            }

            disconnect_slot();
        }

        /**
         * Returns true if the function underlying the Slot is valid.
         */
        explicit operator bool() const
        {
            if (!is_valid_slot)
            {
                return false;
            }

            return is_valid_slot();
        }

    private:
        /**
         * Callback to the Signal to disconnect the Slot.
         */
        Slot<void> disconnect_slot;

        /**
         * Callback to the Signal to check if the Slot is valid.
         */
        Slot<bool> is_valid_slot;
    };

    /**
     * A Signal is a mechanism for communicating via function calls between
     * disparate parts of a code base. A set of functions can be connected to a
     * Signal, and when emit() is called, that set of functions will be
     * executed. This allows for a single producer, multiple consumer paradigm.
     *
     * @tparam R The return type of the connected functions.
     * @tparam Args Variadic list of parameters for the connected functions.
     */
    template <class R, class... Args>
    class Signal: public SignalHandler
    {
    public:
        /**
         * A convenience type for the Slot that this Signal accepts.
         */
        using SlotType = Slot<R, Args...>;

        /**
         * Constructor.
         */
        Signal():
            locked(false),
            next_id(0),
            pending_removal(false),
            slot_records()
        {}

        /**
         * Connect the given Slot to this signal.
         *
         * @param slot The Slot to connect.
         *
         * @return True on success.
         */
        bool connect(const SlotType &slot)
        {
            UINT64 id = 0;
            SacAbortIfNot(insert(slot, id), false);

            return true;
        }

        /**
         * Connect the given Slot to this signal.
         *
         * @param slot The Slot to connect.
         *
         * @return A SignalConnection object that allows for disconnecting the
         *         Slot from this Signal.
         */
        SignalConnection connect_get(const SlotType &slot)
        {
            SignalConnection empty_connection;

            UINT64 id = 0;
            SacAbortIfNot(insert(slot, id), empty_connection);

            return SignalConnection(
                slot_bind(make_slot(*this, &Signal::remove), id),
                slot_bind(make_slot(*this, &Signal::is_valid), id));
        }

        /**
         * Disconnect all Slots from this Signal.
         */
        void clear()
        {
            for (auto &slot_record : slot_records)
            {
                slot_record.request_removal = true;
            }

            pending_removal = true;
            reconcile_removal();
        }

        /**
         * Determine if there are any Slots connected to this Signal.
         *
         * @return True if there is at least one connected Slot.
         */
        bool empty() const
        {
            for (const auto &slot_record : slot_records)
            {
                if (!slot_record.should_remove())
                {
                    return false;
                }
            }

            return true;
        }

        /**
         * Call the connected functions with the given arguments. The given
         * accumulator is used to actually make the function calls and
         * collect the return values.
         *
         * @tparam AccumType The accumulator object that calls the functions and
         *                   collects their return values.
         *
         * @param args The supplied arguments.
         *
         * @return The accumulated return value.
         */
        template <class AccumType = ReturnAccumulator<R, Args...>>
        R emit(Args... args)
        {
            if (locked)
            {
                constexpr bool is_unlocked = false;
                SacAssert(is_unlocked);
                return R();
            }

            locked = true;

            AccumType acc;

            /*
             * Iterate over every Slot record and execute it if it is valid and
             * not in the process of being removed.
             *
             * Note that explicit iterators are used instead of a range-based
             * for loop since the list may be appended to during the iteration.
             */
            for (auto it = slot_records.begin(); it != slot_records.end(); it++)
            {
                slot_record_t &slot_record = *it;

                SacDebugAssert(slot_record.fsptr);

                const std::function<R(Args...)> &f = (*slot_record.fsptr);

                if (slot_record.request_removal || !f)
                {
                    slot_record.request_removal = true;
                    pending_removal = true;
                    continue;
                }

                acc.accumulate(f, args...);
            }

            locked = false;

            reconcile_removal();

            return acc.get_accumulated();
        }

    private:
        /**
         * Remove any Slot records from the list if necessary. This does not
         * have a functional impact, but removes unused/defunct records.
         *
         * @note This will have no effect if called within the context of
         *       emit() until the emissions are complete.
         */
        void reconcile_removal()
        {
            if (!pending_removal)
            {
                return;
            }

            if (locked)
            {
                return;
            }

            /*
             * Remove any Slot records that are requesting removal or are
             * invalid.
             */
            slot_records.remove_if(
                [](const slot_record_t &slot_record) -> bool {
                    return slot_record.should_remove();
                });

            /*
             * Clear the pending_removal flag now that the removal is complete.
             */
            pending_removal = false;
        }

        /**
         * Insert the given Slot into the list.
         *
         * @param slot The Slot to insert.
         * @param[out] id The ID assigned to the Slot.
         *
         * @return True on success.
         */
        bool insert(const SlotType &slot, UINT64 &id)
        {
            SacAbortIfNot(slot, false);
            SacAbortIfNot(slot.fsptr, false);
            SacAbortIfNot((*slot.fsptr), false);

            id = next_id;
            next_id++;
            slot_records.emplace_back(id, slot.fsptr);

            return true;
        }

        /**
         * Remove a Slot record with the given ID.
         *
         * @param id The record ID to search for.
         */
        void remove(const UINT64 id)
        {
            for (auto &slot_record : slot_records)
            {
                if (slot_record.id == id)
                {
                    slot_record.request_removal = true;
                    break;
                }
            }

            pending_removal = true;
            reconcile_removal();
        }

        /**
         * Determine if a Slot record with the given ID exists in this Signal
         * and is valid.
         *
         * @param id The Slot record ID to search for.
         *
         * @return True if a Slot record with the given ID exists and is valid.
         */
        bool is_valid(const UINT64 id) const
        {
            for (const auto &slot_record : slot_records)
            {
                if (slot_record.id == id)
                {
                    return !slot_record.should_remove();
                }
            }

            return false;
        }

        /**
         * A structure to contain a shared_ptr to a function that was taken from
         * a Slot
         */
        struct slot_record_t
        {
            /**
             * Constructor.
             *
             * @param _id The ID.
             * @param _fsptr The shared_ptr to the function.
             */
            slot_record_t(
                const UINT64 _id,
                const std::shared_ptr<std::function<R(Args...)>> &_fsptr):
                id(_id),
                request_removal(false),
                fsptr(_fsptr)
            {}

            /**
             * Determine if this record should be removed.
             *
             * @return True if this record should be removed.
             */
            bool should_remove() const
            {
                SacDebugAssert(fsptr);
                return request_removal || !(*fsptr);
            }

            /**
             * The ID of this record.
             */
            UINT64 id;

            /**
             * True if this record should be removed from the list of records.
             * Once set, the function will not be executed in emit(). However,
             * the actual deletion of the record will not occur immediately if
             * the removal happens within the context of emit().
             */
            bool request_removal;

            /**
             * The shared pointer to the function.
             */
            std::shared_ptr<std::function<R(Args...)>> fsptr;
        };

        /**
         * True if emissions are in progress.
         */
        bool locked;

        /**
         * The ID to be assigned to the next connected Slot record.
         */
        UINT64 next_id;

        /**
         * True if there is a pending request to remove a Slot record. This will
         * be cleared once reconcile_removal() is called outside the context of
         * emit().
         */
        bool pending_removal;

        /**
         * The set of Slot records.
         *
         * Note that it is important that this container remains a list,
         * otherwise the iterator in emit() could be invalidated by connecting
         * new Slots from inside the context of emit().
         */
        std::list<slot_record_t> slot_records;
    };
} /* end namespace Drone */

#endif /* SIGNAL_H */