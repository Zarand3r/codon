/**
 * @author Richard Bao
 *
 * Concrete SlateBuilderStore. One RootState (mutable SlateLayout + shared
 * SlateMemory + accountants) is shared by the root store and every scoped
 * sub-store; each store carries its own subtree/permission/mask/subsystem
 * scoping. Cold path: build-phase only.
 */

#include "src/bullwinkle/all/SlateBuilderStore.h"

#include "src/bullwinkle/all/Slate.h"
#include "src/bullwinkle/all/SlateMemory.h"

#include <utility>
#include <vector>

namespace Drone
{
    namespace
    {
        struct RootState
        {
            Handle<SlateLayout> layout{};
            Handle<SlateMemory> memory{};
            std::vector<SlateTokenAccountantInterface *> accountants{};
            bool finalized = false;
            bool built = false;
        };

        class Store : public SlateBuilderStoreInterface
        {
        public:
            Store(Handle<RootState> _root, std::string _subtree,
                  str_s _masked, const slate_permission_t _permission,
                  const slate_subsystem_id_t _subsystem, const bool _super)
                : root(std::move(_root)), subtree(std::move(_subtree)),
                  masked(std::move(_masked)), permission(_permission),
                  subsystem(_subsystem), super_flag(_super)
            {}

            bool allocate_element(const std::string_view path,
                                  const slate_type_t type_id,
                                  const size_t value_size, const size_t alignment,
                                  const slate_shard_t shard,
                                  const slate_elem_access_t access_policy,
                                  const slate_validator_t &validator,
                                  const slate_subsystem_id_t subsystem_id,
                                  slate_element_t &element_id, void *&mem) override
            {
                FswAbortIf(root->finalized, false);
                return root->layout->allocate_element(
                    path, type_id, value_size, alignment, shard, access_policy,
                    validator, subsystem_id, element_id, mem);
            }

            bool create_view_element(const slate_element_t parent_id,
                                     const slate_type_t parent_type_id,
                                     const size_t element_offset,
                                     const std::string_view path,
                                     const slate_type_t type_id,
                                     const size_t value_size,
                                     const size_t alignment,
                                     const slate_subsystem_id_t subsystem_id,
                                     slate_element_t &view_element_id) override
            {
                FswAbortIf(root->finalized, false);
                return root->layout->create_view_element(
                    parent_id, parent_type_id, element_offset, path, type_id,
                    value_size, alignment, subsystem_id, view_element_id);
            }

            bool bind_write(const std::string_view path,
                            const slate_type_t type_id,
                            slate_element_t &element_id) override
            {
                FswAbortIfNot(
                    root->layout->get_element_id(path, type_id, element_id),
                    false);
                FswAbortIfNot(root->layout->can_write(element_id), false);
                return true;
            }

            bool bind_read(const std::string_view path,
                           const slate_type_t type_id,
                           slate_element_t &element_id) override
            {
                return root->layout->get_element_id(path, type_id, element_id);
            }

            bool get_element_id(const std::string_view path,
                                const slate_type_t type_id,
                                slate_element_t &element_id) const override
            {
                return root->layout->get_element_id(path, type_id, element_id);
            }

            const SlateLayout &slate_layout() const override
            {
                return *root->layout;
            }
            const std::string &get_subtree_path() const override
            {
                return subtree;
            }
            slate_permission_t get_permission() const override
            {
                return permission;
            }
            str_s get_masked_paths() const override { return masked; }
            slate_subsystem_id_t get_subsystem_id() const override
            {
                return subsystem;
            }

            Handle<SlateBuilderStoreInterface>
            sub_slate(const std::string &subtree_path, const str_s &masked_paths,
                      const slate_permission_t sub_permission,
                      const slate_subsystem_id_t subsystem_id) override
            {
                Handle<SlateBuilderStoreInterface> h;
                h.assume_ownership(new Store(root, subtree_path, masked_paths,
                                             sub_permission, subsystem_id,
                                             false));
                return h;
            }

            Handle<SlateBuilderStoreInterface> super_slate() override
            {
                Handle<SlateBuilderStoreInterface> h;
                h.assume_ownership(new Store(root, std::string(), str_s{},
                                             slate_permission_rwc, subsystem,
                                             true));
                return h;
            }
            bool is_super_slate() const override { return super_flag; }

            bool is_peer(
                const Handle<SlateBuilderStoreInterface> &b) const override
            {
                const Store *other = dynamic_cast<const Store *>(b.get());
                return other != nullptr && other->root == root;
            }

            void register_token_accountant(
                SlateTokenAccountantInterface &accountant) override
            {
                for (const auto *a : root->accountants)
                {
                    if (a == &accountant)
                    {
                        return; /* already registered */
                    }
                }
                root->accountants.push_back(&accountant);
            }

            bool finalize() override
            {
                root->finalized = true;
                return true;
            }

            bool build() override
            {
                FswAbortIf(root->built, false);
                root->finalized = true;
                /* Every token handed out must be bound or dismissed. */
                for (auto *a : root->accountants)
                {
                    FswAbortIfNot(a->check_for_uninitialized_tokens(), false);
                    FswAbortIfNot(a->acknowledge_and_disable(), false);
                }
                FswAbortIfNot(root->memory->build(), false);
                root->built = true;
                return true;
            }
            bool is_built() const override { return root->built; }

            Slate slate(const slate_validator_fn_t *validator_fn) override
            {
                if (validator_fn != nullptr)
                {
                    root->memory->validator_sig.connect(*validator_fn);
                }
                return Slate(root->memory);
            }

        private:
            Handle<RootState> root;
            const std::string subtree;
            const str_s masked;
            const slate_permission_t permission;
            const slate_subsystem_id_t subsystem;
            const bool super_flag;
        };
    } /* anonymous namespace */

    Handle<SlateBuilderStoreInterface> CreateSlateBuilderStore()
    {
        Handle<RootState> root;
        root.assume_ownership(new RootState);
        root->layout.assume_ownership(new SlateLayout);
        Handle<const SlateLayout> layout_view(root->layout);
        root->memory.assume_ownership(new SlateMemory(layout_view));

        Handle<SlateBuilderStoreInterface> h;
        h.assume_ownership(new Store(root, std::string(), str_s{},
                                     slate_permission_rwc, 0, false));
        return h;
    }

} /* end namespace Drone */
