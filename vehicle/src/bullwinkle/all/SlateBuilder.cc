/**
 * @author Richard Bao
 * @date   01/28/2025
 */

/*
 * SlateBuilder.h is pre-processed by the static analyzer so it can use the
 * canonical version of Handle<>. Because of this, SlateBuilder.cc cannot be
 * statically analyzed (If it was, it would be using a different version of
 * Handle<> than its header)
 */
#define HOTSYNC_EXEMPT

#include "src/bullwinkle/all/SlateBuilder.h"

#include "src/bullwinkle/all/EnumRegistry.h"

#include <utility>

namespace Drone
{
    /**
     * Constructor.
     */
    SlateBuilder::SlateBuilder():
        store(CreateSlateBuilderStore()),
        layout(store->slate_layout()),
        subtree_path(store->get_subtree_path().c_str()),
        enum_registry_relative_path(),
        enum_registry()
    {
        store->register_token_accountant(slate_token_accountant());

        /*
         * Re-enable the slate token accountant in case we're running in a
         * unit-test.
         */
        slate_token_accountant().enable();
    }

    /**
     * Constructor from a slate store interface instance.
     *
     * @param _store The slate store interface instance to use for creation.
     */
    SlateBuilder::SlateBuilder(Handle<SlateBuilderStoreInterface> _store):
        store(_store),
        layout(store->slate_layout()),
        subtree_path(store->get_subtree_path().c_str()),
        enum_registry_relative_path(),
        enum_registry()
    {
        store->register_token_accountant(slate_token_accountant());
    }

    /**
     * Construct from a slate store interface instance, and add an EnumRegistry.
     *
     * Registering enums with EnumRegistry requires fully-qualified channel
     * names, so a enum_registry_relative_path must be provided as well.
     *
     * @param _store The slate store interface instance to use for creation.
     * @param _enum_registry_relative_path A prefix that the enum registry
     * should add to relative paths before looking up enum names.
     * @param _enum_registry The EnumRegistry to add.
     */
    SlateBuilder::SlateBuilder(Handle<SlateBuilderStoreInterface> _store,
                               const std::string &_enum_registry_relative_path,
                               const Handle<EnumRegistry> &_enum_registry):
        SlateBuilder(_store)
    {
        enum_registry_relative_path = _enum_registry_relative_path;
        enum_registry = _enum_registry;
    }

    /**
     * Copy constructor.
     *
     * @param builder Existing SlateBuilder to be copied from.
     */
    SlateBuilder::SlateBuilder(const SlateBuilder &builder):
        store(builder.store),
        layout(store->slate_layout()),
        subtree_path(store->get_subtree_path().c_str()),
        enum_registry_relative_path(builder.enum_registry_relative_path),
        enum_registry(builder.enum_registry)
    {
        store->register_token_accountant(slate_token_accountant());
    }

    /**
     * Assignment operator.
     *
     * @param builder Existing SlateBuilder to be copied from.
     */
    SlateBuilder SlateBuilder::operator=(const SlateBuilder &builder)
    {
        if (this == &builder)
        {
            return *this;
        }

        /*
         * Because some members are 'const', we need to make the copy using the
         * copy constructor. This means destructing ourselves first, and then
         * invoking the copy constructor via the in-place new operator.
         */
        this->~SlateBuilder();
        new (this) SlateBuilder(builder);
        return *this;
    }

    /**
     * Add an EnumRegistry to this SlateBuilder.
     *
     * SymbolTables added to elements for this slate and its subslates will be
     * registered to this EnumRegistry. Fully-qualified channel names will be
     * relative to this slate.
     *
     * An EnumRegistry cannot be added if one has already been added, either
     * explicitly, or via inheritance of a parent SlateBuilder's EnumRegistry
     *
     * @param _enum_registry The EnumRegistry to add
     * @return True on success
     */
    bool
    SlateBuilder::add_enum_registry(const Handle<EnumRegistry> &_enum_registry)
    {
        FswMsgAbortIf(enum_registry,
                      false,
                      512,
                      "Attempted to add an EnumRegistry to SlateBuilder at '%s'"
                      "which already contains an EnumRegistry",
                      subtree_path.c_str());
        enum_registry = _enum_registry;
        return true;
    }

    /**
     * Add an EnumRegistry and relative path to this SlateBuilder.
     *
     * @param _enum_registry The EnumRegistry to add
     * @param _enum_registry_relative_path A prefix that the enum registry
     * should add to relative paths before looking up enum names.
     * @return True on success
     */
    bool SlateBuilder::add_enum_registry(
        const Handle<EnumRegistry> &_enum_registry,
        const std::string &_enum_registry_relative_path)
    {
        enum_registry_relative_path = _enum_registry_relative_path;
        FswAbortIfNot(add_enum_registry(_enum_registry), false);

        return true;
    }

    /**
     * Check whether this SlateBuilder has an EnumRegistry associated with it.
     *
     * @return True if an EnumRegistry is associated, False if not.
     */
    bool SlateBuilder::has_enum_registry() const
    {
        return !!enum_registry;
    }

    /**
     * Create a sub-slate rooted at a particular part of the subtree.
     *
     * @param rel_subtree_path Root the sub-slate at this point in the tree.
     *
     * @return Sub-slate.
     */
    SlateBuilder SlateBuilder::sub_slate(const std::string &rel_subtree_path)
    {
        const slate_permission_t permission = store->get_permission();
        const str_s masked_paths;
        const slate_subsystem_id_t subsystem_id = store->get_subsystem_id();
        return sub_slate(
            rel_subtree_path, masked_paths, permission, subsystem_id);
    }

    /**
     * Create a sub-slate with sub-paths masked out, and restrict access.
     *
     * @param rel_subtree_path Root the sub-slate at this point in the tree.
     * @param rel_masked_paths Mask out these paths, relative to the new
     *                         \a rel_subtree_path.
     *
     * @return Sub-slate.
     */
    SlateBuilder SlateBuilder::sub_slate(const std::string &rel_subtree_path,
                                         const str_s &rel_masked_paths)
    {
        const slate_permission_t permission = store->get_permission();
        const slate_subsystem_id_t subsystem_id = store->get_subsystem_id();
        return sub_slate(
            rel_subtree_path, rel_masked_paths, permission, subsystem_id);
    }

    /**
     * Create a sub-slate rooted at a particular part of the subtree, and
     * restrict acces.
     *
     * @param rel_subtree_path Root the sub-slate at this point in the tree.
     * @param permission Permissions granted on this Slate.
     *
     * @return Sub-slate.
     */
    SlateBuilder SlateBuilder::sub_slate(const std::string &rel_subtree_path,
                                         const slate_permission_t permission)
    {
        const str_s masked_paths;
        const slate_subsystem_id_t subsystem_id = store->get_subsystem_id();
        return sub_slate(
            rel_subtree_path, masked_paths, permission, subsystem_id);
    }

    /**
     * Create a sub-slate rooted at a particular part of the subtree, and
     * assign all future elements created by it a given subsystem ID.
     *
     * @param rel_subtree_path Root the sub-slate at this point in the tree.
     * @param subsystem_id The subsystem ID to assign to elements created by
     *                     the resulting builder.
     *
     * @return Sub-slate.
     */
    SlateBuilder
    SlateBuilder::sub_slate(const std::string &rel_subtree_path,
                            const slate_subsystem_id_t subsystem_id)
    {
        const slate_permission_t permission = store->get_permission();
        const str_s masked_paths;
        return sub_slate(
            rel_subtree_path, masked_paths, permission, subsystem_id);
    }

    /**
     * Create a sub-slate with sub-paths masked out, and restrict access.
     *
     * @param rel_subtree_path Root the sub-slate at this point in the tree.
     * @param rel_masked_paths Mask out these paths, relative to the new
     *                         \a rel_subtree_path.
     * @param permission Permissions granted on this Slate.
     * @param subsystem_id The subsystem ID to assign to elements created by
     *                     the resulting builder.
     *
     * @return Sub-slate.
     */
    SlateBuilder
    SlateBuilder::sub_slate(const std::string &rel_subtree_path,
                            const str_s &rel_masked_paths,
                            const slate_permission_t permission,
                            const slate_subsystem_id_t subsystem_id)
    {
        const slate_permission_t parent_permission = store->get_permission();
        const str_s masked_paths = store->get_masked_paths();

        if (FswIfNot(
                slate_permission_is_superset(parent_permission, permission)))
        {
            FswPrefix();
            dbstring(": Cannot elevate privileges in a sub-slate.\n");
            FswAssert(false);
        }

        /*
         * Check to see if we're trying to create a forbidden path.
         */
        for (str_s::const_iterator iter = masked_paths.begin();
             iter != masked_paths.end();
             ++iter)
        {
            std::string_view stripped;
            if (slate_rel_path(*iter, rel_subtree_path, stripped))
            {
                FswPrefix();
                dbnprintf(500,
                          ": Cannot create sub-slate at '%s', path '%s' "
                          "is forbidden.\n",
                          rel_subtree_path.c_str(),
                          iter->c_str());
                FswAssert(false);
            }
        }

        str_s new_masked_paths;

        const std::string new_subtree_path =
            slate_join_path(subtree_path, rel_subtree_path);

        /*
         * Add the new paths with our subtree prepended.
         */
        for (str_s::const_iterator iter = rel_masked_paths.begin();
             iter != rel_masked_paths.end();
             ++iter)
        {
            new_masked_paths.insert(slate_join_path(new_subtree_path, *iter));
        }

        /*
         * Rewrite the existing masked paths, discarding subtrees outside
         * the new root.
         */
        for (str_s::const_iterator iter = masked_paths.begin();
             iter != masked_paths.end();
             ++iter)
        {
            std::string_view stripped;
            if (slate_rel_path(new_subtree_path, *iter, stripped))
            {
                /*
                 * Keep the fully-qualified path.
                 */
                new_masked_paths.insert(*iter);
            }
        }

        const slate_permission_t new_permission =
            slate_permission_take_subset(parent_permission, permission);

        if (enum_registry)
        {
            return SlateBuilder(store->sub_slate(new_subtree_path,
                                                 new_masked_paths,
                                                 new_permission,
                                                 subsystem_id),
                                slate_join_path(enum_registry_relative_path,
                                                rel_subtree_path),
                                enum_registry);
        }
        else
        {
            return SlateBuilder(store->sub_slate(new_subtree_path,
                                                 new_masked_paths,
                                                 new_permission,
                                                 subsystem_id));
        }
    }

    /**
     * Get a copy of the Slate, which can be used at run-time to load and
     * store element data.
     *
     * You are required to provide a validator function. The validator
     * function should examine the Slate object that was returned to you
     * and check any data stored in the static shard for consistency. If
     * there is some problem with the static data or any data derived
     * from it, you may either modify the data to be self-consistent or
     * return false to reject the change as nonsensical.
     *
     * If any component returns false during validation, all changes made
     * during the validation period by every component will be rolled
     * back and no change to the Slate will occur.
     *
     * validator_fn will be called when build() is called the first time,
     * and if data in the static shard is ever modified.
     *
     * If you do not wish to perform any validation, pass in the
     * #slate_no_validation slot.
     *
     * This Slate will not work until build() is called.
     *
     * @param validator_fn This function will be called on build(), or
     *                     when the static shard is modified. Returning
     *                     false from this function indicates that the
     *                     Slate state is not valid and should be
     *                     rejected.
     *
     * @return Slate copy.
     */
    Slate SlateBuilder::slate(const slate_validator_fn_t &validator_fn) const
    {
        /*
         * Don't waste time calling the null validator. We need to perform the
         * comparison check here since the store might be living in a different
         * module where slate_no_validation has a different value.
         */
        if (&validator_fn == &slate_no_validation)
        {
            return store->slate(NULL);
        }
        else
        {
            return store->slate(&validator_fn);
        }
    }

    /**
     * Returns true if \a b is a "peer" SlateBuilder - that is, they
     * point to the same SlateMemory object. If two SlateBuilders are
     * peers, then it is safe to get a single Slate run-time object for
     * both of them.
     *
     * @param b Check this slate builder.
     *
     * @retun True if the two builders are peers.
     */
    bool SlateBuilder::is_peer(const SlateBuilder b) const
    {
        return store->is_peer(b.store);
    }

    /**
     * Registers an enum for a Slate element using its path.
     * An EnumRegistry must be added to the SlateBuilder, either directly,
     * or via inheritance of a parent SlateBuilder's EnumRegistry
     *
     * @param element_path The path to the element.
     * @param enum_name The name of the enum.
     * @param symbol_table The SymbolTable that describes the enum.
     * @param strip_prefix If this is a prefix in the enumerated values,
     *                     strip it away. Useful for making auto_enum
     *                     SymbolTable's more readable.
     * @return True on success.
     */
    bool SlateBuilder::register_enum(const std::string &element_path,
                                     const std::string &enum_name,
                                     const SymbolTable &symbol_table,
                                     const std::string &strip_prefix)
    {
        const std::string channel_name =
            slate_join_path(enum_registry_relative_path, element_path);
        FswMsgAbortIfNot(
            enum_registry,
            false,
            256,
            "Cannot register enum '%s' in SlateBuilder at '%s' without "
            "an EnumRegistry",
            enum_name.c_str(),
            subtree_path.c_str());
        FswAbortIfNot(enum_registry->register_enum(
                          enum_name, symbol_table, channel_name, strip_prefix),
                      false);
        return true;
    }

    /**
     * Get the ID of a Slate element.
     *
     * @param element_path Get the element at this path.
     * @param type_id The expected type ID of this element.
     * @param[out] element_id Returns the element ID.
     *
     * @return True on success.
     */
    bool SlateBuilder::get_element_id(const std::string_view element_path,
                                      const slate_type_t type_id,
                                      slate_element_t &element_id) const
    {
        const std::string full_path =
            slate_join_path(subtree_path, element_path);
        FswAbortIfNot(require_permitted_path(full_path), false);

        FswAbortIfNot(store->get_element_id(full_path, type_id, element_id),
                      false);

        return true;
    }

    /**
     * Get the type of an element, given its path.
     *
     * @param element_path Path to the element.
     * @param[out] type_id Returns the element's type ID.
     *
     * @param True on success.
     */
    bool SlateBuilder::get_element_type(const std::string &element_path,
                                        slate_type_t &type_id) const
    {
        const std::string full_path =
            slate_join_path(subtree_path, element_path);
        FswAbortIfNot(require_permitted_path(full_path), false);

        return layout.get_element_type(full_path, type_id);
    }

    /**
     * Get an element's enum name and symbol table, if any, given its path.
     *
     * @param element_path Path to the element.
     * @param[out] enum_exists Whether an enum could be found
     * @param[out] enum_name The name of the enum.
     * @param[out] symbol_table The symbol table for the enum.
     *
     * @return True on success.
     */
    bool SlateBuilder::get_element_enum(const std::string &element_path,
                                        bool &enum_exists,
                                        std::string &enum_name,
                                        SymbolTable &symbol_table) const
    {
        const std::string channel_name =
            slate_join_path(enum_registry_relative_path, element_path);
        FswMsgAbortIfNot(
            enum_registry,
            false,
            256,
            "Cannot get_element_enum in SlateBuilder at '%s' without "
            "an EnumRegistry",
            subtree_path.c_str());
        enum_exists = enum_registry->get_registered_enum(channel_name,
                                                         enum_name,
                                                         symbol_table);
        return true;
    }

    /**
     * Request the slate_type_info_t corresponding to a slate type ID. This
     * function returns a pointer to the information requested to avoid any
     * allocation.
     *
     * @param type The slate type to get information about.
     * @param[out] info The information requested.
     *
     * @return True if type is valid.
     */
    bool SlateBuilder::get_type_info(slate_type_t type,
                                     const slate_type_info_t *&info) const
    {
        return slate_type_info_utils::get_type_info(type, info);
    }

    /**
     * Get the full path of an element, given its ID. If this is a sub_slate,
     * the path is relative to the root slate, not the sub_slate. If there are
     * multiple Slate elements corresponding to this ID and type, it returns the
     * path of the element that was created first.
     *
     * @param element_id ID of the element.
     * @param type_id Type ID of the element.
     * @param[out] path Returns the element's path.
     *
     * @return True on success.
     */
    bool SlateBuilder::get_first_element_path(const slate_element_t element_id,
                                              const slate_type_t type_id,
                                              std::string &path) const
    {
        std::string_view first_path;
        FswAbortIfNot(
            layout.get_first_element_path(element_id, type_id, first_path),
            false);

        path = first_path;
        return true;
    }

    /**
     * Returns true if an element with the given path exists.
     *
     * @param element_path Check for an element at this path.
     *
     * @return True if there is an element at this path.
     */
    bool SlateBuilder::path_exists(const std::string &element_path) const
    {
        const std::string full_path =
            slate_join_path(subtree_path, element_path);
        if (!is_permitted_path(full_path))
        {
            return false;
        }

        return layout.path_exists(full_path);
    }

    /**
     * Returns true if an element with the given path and type ID exists.
     *
     * @param element_path Check for an element at this path.
     * @param type_id Check against this type.
     *
     * @return True if path exists and type matches.
     */
    bool SlateBuilder::element_exists(const std::string &element_path,
                                      const slate_type_t type_id) const
    {
        const std::string full_path =
            slate_join_path(subtree_path, element_path);
        if (!is_permitted_path(full_path))
        {
            return false;
        }

        return layout.element_exists(full_path, type_id);
    }

    /**
     * Check for write privileges.
     *
     * @param element_id Check this Slate ID.
     *
     * @return True if \a element_id has write privileges.
     */
    bool SlateBuilder::can_write(const slate_element_t element_id) const
    {
        return layout.can_write(element_id);
    }

    /**
     * Check if an element has a validator.
     *
     * @param element_id Check this Slate ID.
     *
     * @return True if \a element_id has a validator.
     */
    bool SlateBuilder::must_validate(const slate_element_t element_id) const
    {
        return layout.must_validate(element_id);
    }

    /**
     * Get the full path relative to the root Slate for a relative path.
     *
     * @param relative_path Path relative to the current builder.
     *
     * @return Full path relative to root Slate.
     */
    std::string
    SlateBuilder::get_absolute_path(const std::string &relative_path) const
    {
        return slate_join_path(get_subtree_path(), relative_path);
    }

    /**
     * Get the path relative to this builder for a full path.
     *
     * @param absolute_path Path relative to the root Slate.
     * @param[out] relative_path Path relative to the current builder.
     *
     * @return True if the absolute path is a subpath (or equal to) this
     *         builder's subtree path.
     */
    bool SlateBuilder::get_relative_path(const std::string &absolute_path,
                                         std::string &relative_path) const
    {
        return slate_rel_path(get_subtree_path(), absolute_path, relative_path);
    }

    /**
     * Returns true if the Slate is built.
     *
     * @return True if the Slate is built.
     */
    bool SlateBuilder::is_built() const
    {
        return store->is_built();
    }

    /**
     * Build the real slate from the created devices. No further creates
     * will be allowed, and the Slate instances will now function.
     *
     * @return True on success.
     */
    bool SlateBuilder::build()
    {
        return store->build();
    }

    /**
     * Finalize the slate, cleaning up any build time metadata.
     *
     * @return True on success.
     */
    bool SlateBuilder::finalize()
    {
        return store->finalize();
    }

    /**
     * Returns true if this is a "super" SlateBuilder.
     *
     * @return True if this is a "super" SlateBuilder.
     */
    bool SlateBuilder::is_super_slate() const
    {
        return store->is_super_slate();
    }

    /**
     * Creates a "super" SlateBuilder which ignores all permissions and
     * grants read/write access to all elements. Used for external tools,
     * such as command and telemetry.
     *
     * Super SlateBuilders can only be created from the root SlateBuilder.
     *
     * @return A super SlateBuilder.
     */
    SlateBuilder SlateBuilder::super_slate()
    {
        if (is_super_slate())
        {
            return *this;
        }

        Handle<SlateBuilderStoreInterface> super_store = store->super_slate();
        FswAssert(super_store);
        return SlateBuilder(super_store);
    }

    /**
     * Returns the backing slate store of this SlateBuilder.
     *
     * @return The backing slate store.
     */
    Handle<SlateBuilderStoreInterface> SlateBuilder::get_store()
    {
        return store;
    }

    /**
     * Compute the full path -> element id set for non-private elements
     * accessibly in this Slate.
     *
     * @param[out] paths Returns the path -> id mapping.
     *
     * @return True on success.
     */
    bool SlateBuilder::compute_path_set(slateelem_v &paths) const
    {
        const bool is_superslate = is_super_slate();

        paths.reserve(layout.get_elements().size());
        for (const auto &[path, metadata] : layout.get_elements())
        {
            const slate_elem_access_t access =
                is_superslate ? slate_read_write : metadata.access_policy;

            if (slate_private == access)
            {
                continue;
            }

            slate_element_key_t out_elem;
            out_elem.type_id = metadata.type_id;

            if (!slate_rel_path(subtree_path, path, out_elem.path))
            {
                continue;
            }

            FswAbortIfNot(layout.build_element_id(
                              path, metadata, access, out_elem.element_id),
                          false);

            paths.emplace_back(std::move(out_elem));
        }

        return true;
    }

    /**
     * Returns the subtree of the Slate where this SlateBuilder is "rooted".
     * All elements created or bound will have this path prepended to
     * them.
     *
     * @return The prefix for this sub-Slate, empty string for root Slate.
     */
    const std::string &SlateBuilder::get_subtree_path() const
    {
        return subtree_path;
    }

    /**
     * Check that there are no collisions between the names of any of the
     * Slate elements across all the SlateBuilders provided.
     *
     * @param sudo_builders The list of SlateBuilders to check. These must be
     *                      sudo SlateBuilders that have already been built.
     *
     * @return True on success.
     */
    bool
    SlateBuilder::names_distinct(const std::vector<SlateBuilder> &sudo_builders)
    {
        FswAbortIf(sudo_builders.empty(), false);

        str_s names;

        for (const auto &builder : sudo_builders)
        {
            /*
             * Make sure this is a built super SlateBuilder. This is to enforce
             * that all elements are created and we can access all of them.
             */
            FswAbortIfNot(builder.is_built(), false);
            FswAbortIfNot(builder.is_super_slate(), false);

            /*
             * Lookup all the elements in this builder.
             */
            slateelem_v path_set;
            FswAbortIfNot(builder.compute_path_set(path_set), false);

            /*
             * Iterate over every element and add its name to the set.
             */
            for (const auto &elem : path_set)
            {
                const auto insert_ret = names.insert(elem.path);

                if (!insert_ret.second)
                {
                    FswMsgAbort(false,
                                200,
                                "Duplicate element name '%s' in subtree "
                                "path '%s'.",
                                elem.path.c_str(),
                                builder.get_subtree_path().c_str());
                }
            }
        }

        return true;
    }

    /**
     * Returns true if the given fully-qualified path is permitted in
     * this slate, or false if it descends from a masked subtree.
     *
     * @param full_path The fully-qualified path.
     *
     * @return True if permitted.
     */
    bool SlateBuilder::is_permitted_path(const std::string &full_path) const
    {
        /*
         * Check all of our masks to see if the path is part of a
         * masked subtree.
         */
        const str_s &masked_paths = store->get_masked_paths();
        str_s::const_iterator iter;
        for (iter = masked_paths.begin(); iter != masked_paths.end(); ++iter)
        {
            std::string_view stripped;
            if (slate_rel_path(*iter, full_path, stripped))
            {
                return false;
            }
        }

        return true;
    }

    /**
     * Returns true if the given fully-qualified path is permitted in
     * this slate, or false if it descends from a masked subtree.
     *
     * Prints an error if the path is forbidden.
     *
     * @param full_path The fully-qualified path.
     *
     * @return True if permitted.
     */
    bool
    SlateBuilder::require_permitted_path(const std::string &full_path) const
    {
        const str_s &masked_paths = store->get_masked_paths();
        str_s::const_iterator iter;
        for (iter = masked_paths.begin(); iter != masked_paths.end(); ++iter)
        {
            std::string_view stripped;
            if (slate_rel_path(*iter, full_path, stripped))
            {
                FswPrefix();
                dbnprintf(200,
                          ": Path '%s' is forbidden; masked by '%s'.\n",
                          full_path.c_str(),
                          iter->c_str());
                return false;
            }
        }

        return true;
    }

} /* end namespace Drone */