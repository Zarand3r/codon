/**
 * @author Richard Bao
 *
 * SlateBuilderStoreInterface — where a SlateBuilder's data actually lives.
 *
 * SlateBuilder is a copyable, subtree-scoped *view*; every builder for the same
 * Slate shares one store through a Handle. The store owns the mutable SlateLayout
 * during the build phase, carries the builder's scoping state (subtree path,
 * permission, masked paths, subsystem id), vends scoped child stores
 * (sub_slate/super_slate), and — once building completes — materializes and hands
 * out the runtime Slate. Interface + concrete-store factory
 * (CreateSlateBuilderStore) inferred from every `store->` call site in
 * SlateBuilder.{h,cc}.
 *
 * Cold path: everything here is build-phase; the runtime Slate never touches the
 * store again after build().
 */

#ifndef SLATE_BUILDER_STORE_H
#define SLATE_BUILDER_STORE_H

#include "src/bullwinkle/all/Handle.h"
#include "src/bullwinkle/all/Signal.h"
#include "src/bullwinkle/all/SlateLayout.h"
#include "src/bullwinkle/all/core/util.h"
#include "src/bullwinkle/all/slate_tokens.h"

#include <string>
#include <string_view>

namespace Drone
{
    class Slate;
    typedef Slot<bool> slate_validator_fn_t;

    /** Result of a builder create/bind (bool today; kept nominal so call sites
     *  read `result_t` as the imported API does). */
    typedef bool result_t;

    class SlateBuilderStoreInterface
    {
    public:
        virtual ~SlateBuilderStoreInterface() {}

        /* ---- element creation / binding (delegates into the SlateLayout) ---- */

        virtual bool allocate_element(const std::string_view path,
                                      const slate_type_t type_id,
                                      const size_t value_size,
                                      const size_t alignment,
                                      const slate_shard_t shard,
                                      const slate_elem_access_t access_policy,
                                      const slate_validator_t &validator,
                                      const slate_subsystem_id_t subsystem_id,
                                      slate_element_t &element_id,
                                      void *&mem) = 0;

        virtual bool create_view_element(const slate_element_t parent_id,
                                         const slate_type_t parent_type_id,
                                         const size_t element_offset,
                                         const std::string_view path,
                                         const slate_type_t type_id,
                                         const size_t value_size,
                                         const size_t alignment,
                                         const slate_subsystem_id_t subsystem_id,
                                         slate_element_t &view_element_id) = 0;

        /** Resolve an existing path to a writable element id (fails if the
         *  element is not writable). */
        virtual bool bind_write(const std::string_view path,
                                const slate_type_t type_id,
                                slate_element_t &element_id) = 0;

        /** Resolve an existing path to a readable element id. */
        virtual bool bind_read(const std::string_view path,
                               const slate_type_t type_id,
                               slate_element_t &element_id) = 0;

        virtual bool get_element_id(const std::string_view path,
                                    const slate_type_t type_id,
                                    slate_element_t &element_id) const = 0;

        /* ---- scoping state ---- */

        virtual const SlateLayout &slate_layout() const = 0;
        virtual const std::string &get_subtree_path() const = 0;
        virtual slate_permission_t get_permission() const = 0;
        virtual str_s get_masked_paths() const = 0;
        virtual slate_subsystem_id_t get_subsystem_id() const = 0;

        /** A child store scoped to `subtree_path` with narrowed rights. */
        virtual Handle<SlateBuilderStoreInterface>
        sub_slate(const std::string &subtree_path, const str_s &masked_paths,
                  const slate_permission_t permission,
                  const slate_subsystem_id_t subsystem_id) = 0;

        /** The permission-exempt root-scoped store (tools only). */
        virtual Handle<SlateBuilderStoreInterface> super_slate() = 0;
        virtual bool is_super_slate() const = 0;

        /** True if `b` shares this store's underlying Slate (same root state). */
        virtual bool is_peer(const Handle<SlateBuilderStoreInterface> &b) const = 0;

        /* ---- lifecycle ---- */

        virtual void
        register_token_accountant(SlateTokenAccountantInterface &accountant) = 0;

        /** Freeze the layout: no further create/bind after this. */
        virtual bool finalize() = 0;

        /** Materialize the runtime memory (copy initial values into the live
         *  shards). Requires finalize(); checks token accountants. */
        virtual bool build() = 0;
        virtual bool is_built() const = 0;

        /**
         * A runtime Slate over this store's (shared) memory. Valid to call before
         * build(); the Slate works once build() succeeds. `validator_fn` (may be
         * NULL) is invoked on build and on static-shard modification.
         */
        virtual Slate slate(const slate_validator_fn_t *validator_fn) = 0;
    };

    /** The root store for a fresh Slate (full permission, empty subtree). */
    Handle<SlateBuilderStoreInterface> CreateSlateBuilderStore();

} /* end namespace Drone */

#endif /* SLATE_BUILDER_STORE_H */
