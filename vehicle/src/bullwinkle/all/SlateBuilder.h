/**
 * @author Richard Bao
 * @date   01/28/2025
 */

#ifndef SLATE_BUILDER_H
#define SLATE_BUILDER_H

#include "src/bullwinkle/all/EnumRegistry.h"
#include "src/bullwinkle/all/Handle.h"
#include "src/bullwinkle/all/Signal.h"
#include "src/bullwinkle/all/Slate.h"
#include "src/bullwinkle/all/SlateBuilderStore.h"
#include "src/bullwinkle/all/SlateLayout.h"
#include "src/bullwinkle/all/SlateMemory.h"
#include "src/bullwinkle/all/slate_accessor.h"

namespace Drone {
struct slate_element_key_t;
using slateelem_v = std::vector<slate_element_key_t>;

/**
 * Tool for building a Slate by passing SlateBuilder references
 * around and allowing software systems to create Slate data.
 *
 * For high-level documentation, see Slate.
 *
 *
 * BUILD PHASE:
 *
 * The SlateBuilder is in charge of the build phase. This is when all
 * elements are created, permissions are set, and memory layout is
 * determined.
 *
 * Once the build phase is complete, no modification of the Slate
 * structure is permitted. This enables several optimizations at run
 * time.
 *
 *
 * RUN PHASE:
 *
 * For information on the run phase, see Slate.
 *
 *
 * SHARDS:
 *
 * Elements are addressed by their location in the logical layout,
 * but their storage location is determined by their shard. Shards
 * define the "memory policy" for a particular element. Note that
 * shards are completely independent from the logical layout - two
 * elements in the same namespace can be in completely different
 * shards with totally disparate memory policies.
 *
 * See the slate_shard_t enum for information on specific shards.
 *
 *
 * PERMISSIONS:
 *
 * The SlateBuilder is responsible for assigning and checking
 * permissions on elements. There are two levels of permissions.
 *
 * Each element has a permission setting which defines the
 * permissions granted to other sub-systems (excluding command and
 * telemetry). These settings are explained in the slate_permission_t
 * enum.
 *
 * Additionally, each SlateBuilder (or sub-SlateBuilder) has a
 * permissions mask, which can restrict (but not elevate) the
 * element-level permissions. In this way, higher-level systems can
 * programatically determine the overall level of control lower-level
 * systems have, much as choosing between const and non-const
 * references can. This mask is explained in the slate_permission_t
 * enum.
 */
class SlateBuilder {
public:
  SlateBuilder();
  SlateBuilder(Handle<SlateBuilderStoreInterface> _store);
  SlateBuilder(Handle<SlateBuilderStoreInterface> _store,
               const std::string &enum_registry_relative_path,
               const Handle<EnumRegistry> &enum_registry);
  SlateBuilder(const SlateBuilder &builder);
  SlateBuilder operator=(const SlateBuilder &builder);

  SlateBuilder sub_slate(const std::string &rel_subtree_path);

  SlateBuilder sub_slate(const std::string &rel_subtree_path,
                         const str_s &rel_masked_paths);

  SlateBuilder sub_slate(const std::string &rel_subtree_path,
                         const slate_permission_t permission);

  SlateBuilder sub_slate(const std::string &rel_subtree_path,
                         const slate_subsystem_id_t subsystem_id);

  bool add_enum_registry(const Handle<EnumRegistry> &_enum_registry);
  bool add_enum_registry(const Handle<EnumRegistry> &_enum_registry,
                         const std::string &enum_registry_relative_path);
  bool has_enum_registry() const;

  /*
   * Get a copy of the run-time interface.
   */
  /* NOTE: "no validation" is recognized by ADDRESS — pass the canonical
   * `slate_no_validation` object; an equal-but-distinct empty slot is treated
   * as a real validator. */
  Slate slate(const slate_validator_fn_t &validator_fn) const;
  bool is_peer(const SlateBuilder b) const;

  /*
   * Create elements without element_id output param.
   */
  template <typename T>
  [[nodiscard]] bool
  create_read_only_element(const std::string &element_path,
                           typename slate_info<T>::R initial_value,
                           const slate_shard_t shard);

  /*
   * Create elements by token.
   */
  template <typename T, int Access>
  result_t create(const std::string &element_path, const slate_shard_t shard,
                  SlateAccessToken<T, Access> &token);

  template <typename T, int Access>
  result_t create(const std::string &element_path,
                  typename slate_info<T>::R initial_value,
                  const slate_shard_t shard,
                  SlateAccessToken<T, Access> &token);

  template <typename T, int Access>
  result_t create(const std::string &element_path, const slate_shard_t shard,
                  const slate_elem_access_t access,
                  SlateAccessToken<T, Access> &token);

  template <typename T, int Access>
  result_t create(const std::string &element_path,
                  typename slate_info<T>::R initial_value,
                  const slate_shard_t shard, const slate_elem_access_t access,
                  SlateAccessToken<T, Access> &token);

  template <typename T, int Access>
  result_t create(const std::string &element_path,
                  typename slate_info<T>::R initial_value,
                  const slate_shard_t shard, const slate_elem_access_t access,
                  const slate_validator_t &validator,
                  SlateAccessToken<T, Access> &token);

  /*
   * Create elements by token and register an enum.
   */
  template <class Enum_T, typename T, int Access>
  result_t create_with_enum(const std::string &element_path,
                            const slate_shard_t shard,
                            SlateAccessToken<T, Access> &token);

  template <class Enum_T, typename T, int Access>
  result_t create_with_enum(const std::string &element_path,
                            typename slate_info<T>::R initial_value,
                            const slate_shard_t shard,
                            SlateAccessToken<T, Access> &token);

  template <class Enum_T, typename T, int Access>
  result_t create_with_enum(const std::string &element_path,
                            const slate_shard_t shard,
                            const slate_elem_access_t access,
                            SlateAccessToken<T, Access> &token);

  template <class Enum_T, typename T, int Access>
  result_t create_with_enum(const std::string &element_path,
                            typename slate_info<T>::R initial_value,
                            const slate_shard_t shard,
                            const slate_elem_access_t access,
                            SlateAccessToken<T, Access> &token);

  template <class Enum_T, typename T, int Access>
  result_t create_with_enum(const std::string &element_path,
                            typename slate_info<T>::R initial_value,
                            const slate_shard_t shard,
                            const slate_elem_access_t access,
                            const slate_validator_t &validator,
                            SlateAccessToken<T, Access> &token);

  /*
   * Bind elements by token.
   */
  template <typename T, int Access>
  result_t bind(const std::string &element_path,
                SlateAccessToken<T, Access> &token) const;

  template <class Enum_T, typename T, int Access>
  result_t bind_with_enum(const std::string &element_path,
                          SlateAccessToken<T, Access> &token);

  /*
   * Create view elements by parent string and path.
   */
  template <typename T>
  bool create_view(const std::string &parent_element, SlateBuilder parent_path,
                   const std::string &element, size_t element_offset = 0);

  /*
   * Create view elements by parent token.
   */
  template <typename T, typename ParentT, int ParentAccess>
  bool create_view(SlateAccessToken<ParentT, ParentAccess> &parent_token,
                   const std::string &element_path,
                   const size_t element_offset);

  template <typename T, typename ParentT, int ParentAccess>
  bool create_view(SlateAccessToken<ParentT, ParentAccess> &parent_token,
                   const std::string &element_path, const size_t element_offset,
                   SlateAccessToken<T, ParentAccess> &token);

  /*
   * Create view elements by parent ID.
   */
  template <typename T>
  bool create_view(const slate_element_t parent_element_id,
                   const slate_type_t parent_type_id,
                   const std::string &element_path,
                   const size_t element_offset);

  template <typename T>
  bool create_view(const slate_element_t parent_element_id,
                   const slate_type_t parent_type_id,
                   const std::string &element_path, const size_t element_offset,
                   slate_element_t &view_element_id);

  /*
   * Create elements by ID.
   */
  template <typename T>
  bool create_element(const std::string &element_path,
                      typename slate_info<T>::R initial_value,
                      const slate_shard_t shard,
                      const slate_elem_access_t access,
                      slate_element_t &element_id);

  template <typename T>
  bool create_element(const std::string &element_path,
                      typename slate_info<T>::R initial_value,
                      const slate_shard_t shard,
                      const slate_elem_access_t access,
                      const slate_validator_t &validator,
                      slate_element_t &element_id);

  /*
   * create_element() won't get a non-templated version until
   * pointers are banned from Slate.
   */

  bool register_enum(const std::string &element_path,
                     const std::string &enum_name,
                     const SymbolTable &symbol_table,
                     const std::string &strip_prefix = "");

  template <typename T>
  bool register_enum(const SlateToken<T> &token, const std::string &enum_name,
                     const SymbolTable &symbol_table,
                     const std::string &strip_prefix = "");

  template <typename T>
  bool get_element_id(const std::string_view element_path,
                      slate_element_t &element_id) const;

  bool get_element_id(const std::string_view element_path,
                      const slate_type_t type_id,
                      slate_element_t &element_id) const;

  bool get_first_element_path(const slate_element_t element_id,
                              const slate_type_t type_id,
                              std::string &path) const;

  bool get_element_type(const std::string &element_path,
                        slate_type_t &type_id) const;

  bool get_element_enum(const std::string &element_path, bool &enum_exists,
                        std::string &enum_name,
                        SymbolTable &symbol_table) const;

  bool get_type_info(slate_type_t type, const slate_type_info_t *&info) const;

  /**
   * Return the slate validator associated with the element ID.
   *
   * @param element_id ID of the element to get the validator for.
   * @param type_id Type ID of the element.
   *
   * @return The validator for the element or a default slate_validator_t
   *         if none is available.
   */
  slate_validator_t get_element_validator(const slate_element_t element_id,
                                          const slate_type_t type_id) const {
    return layout.get_element_validator(element_id, type_id);
  }

  template <typename T>
  bool element_exists(const std::string &element_path) const;

  bool element_exists(const std::string &element_path,
                      const slate_type_t type_id) const;

  bool path_exists(const std::string &element_path) const;

  bool can_write(const slate_element_t element_id) const;
  bool must_validate(const slate_element_t element_id) const;

  /*
   * Get the initial value previously provided to create().
   */

  template <typename T>
  bool get_initial_value(const SlateToken<T> &token,
                         typename slate_info<T>::O value) const;

  template <typename T>
  bool get_element_initial_value(const slate_element_t elem_id,
                                 typename slate_info<T>::O value) const;

  template <typename T>
  bool get_path(const SlateToken<T> &token, std::string &path) const;

  std::string get_absolute_path(const std::string &relative_path) const;
  bool get_relative_path(const std::string &absolute_path,
                         std::string &relative_path) const;

  /*
   * Called once everyone has had their chance to create elements.
   */
  bool is_built() const;
  bool build();
  bool finalize();

  bool is_super_slate() const;
  SlateBuilder super_slate();

  /*
   * Useful to connect SlateBuilder instances across modules and interface
   * boundaries.
   */
  Handle<SlateBuilderStoreInterface> get_store();

  /*
   * For specific tools that operate on Slate - should not be used
   * by normal users.
   */
  bool compute_path_set(slateelem_v &paths) const;
  const std::string &get_subtree_path() const;
  static bool names_distinct(const std::vector<SlateBuilder> &sudo_builders);

private:
  template <typename T, int Access>
  bool register_token(slate_element_t element_id,
                      const std::string &element_path,
                      const slate_validator_t &validator,
                      SlateAccessToken<T, Access> &token);

  SlateBuilder sub_slate(const std::string &rel_subtree_path,
                         const str_s &rel_masked_paths,
                         const slate_permission_t permission,
                         const slate_subsystem_id_t subsystem_id);

  bool is_permitted_path(const std::string &full_path) const;
  bool require_permitted_path(const std::string &full_path) const;

  /**
   * Explicitly prevents calling create() incorrectly. This catches cases
   * where the wrong function (access is used as initialization value) is
   * called because the caller incorrectly swapped the order of the
   * function parameters.
   */
  template <typename T, int Access>
  result_t create(const std::string &element_path,
                  const slate_elem_access_t access, const slate_shard_t shard,
                  SlateAccessToken<T, Access> &token) = delete;

  /**
   * Pointer to the SlateBuilder store. This is where all data are
   * actually stored.
   */
  Handle<SlateBuilderStoreInterface> store;

  /**
   * SlateLayout access cache, retrieved from the SlateBuilder store.
   */
  const SlateLayout &layout;

  /**
   * subtree_path cache, retrieved from the SlateBuilder store.
   */
  const std::string subtree_path;

  /**
   * Path of this SlateBuilder relative to the telemetry root.
   */
  std::string enum_registry_relative_path;

  /**
   * Optional EnumRegistry for querying and registering enums associated
   * with elements within this SlateBuilder.
   */
  Handle<EnumRegistry> enum_registry;
};

/**
 * Records the path, ID, and type of a Slate element.
 */
struct slate_element_key_t {
  /**
   * The path of the element.
   */
  std::string path;

  /**
   * The type ID of the element.
   */
  slate_type_t type_id;

  /**
   * The ID of the element.
   */
  slate_element_t element_id;
};

/*
 * Verifies the given element ID, path, and validator are acceptable for the
 * given token, and if so, registers the token and sets its ID.
 *
 * @tparam T The element data type.
 * @tparam Access A combination of slate_token_access_t values.
 *
 * @param element_id ID of the element to use with this token.
 * @param element_path Create the element at this path.
 * @param validator The on-write validator to use.
 * @param[out] token Returns a token to the element.
 *
 * @return True on success.
 */
template <typename T, int Access>
bool SlateBuilder::register_token(slate_element_t element_id,
                                  const std::string &element_path,
                                  const slate_validator_t &validator,
                                  SlateAccessToken<T, Access> &token) {
  FswAbortIfNot(slate_id_is_valid(element_id), false);

  using Token = SlateAccessToken<T, Access>;

  if (Token::can_write) {
    if (!layout.can_write(element_id)) {
      FswPrefix();
      dbnprintf(200, ": New element '%s' is not write-accessible.\n",
                element_path.c_str());
      return false;
    }

    /*
     * If a validator is provided, we need a token that does validation.
     * If a token that does validation is provided but there isn't a
     * validation function, we also fail to force callers to use a
     * WriteToken which is more efficient.
     */
    if (Token::can_validate == validator.is_noop()) {
      FswPrefix();
      dbnprintf(500, ": Creating '%s' element requires a %s.\n",
                element_path.c_str(),
                Token::can_validate ? "WriteToken" : "WriteValidatorToken");
      return false;
    }
  } else {
    element_id = slate_id_ro(element_id);
    FswAbortIf(layout.can_write(element_id), false);
  }

  const std::string full_path = slate_join_path(subtree_path, element_path);
  FswAbortIfNot(slate_token_accountant().register_id(token.id, full_path),
                false);

  token.id = element_id;

  return true;
}

/**
 * Create an element with read only access.
 *
 * This is ideal for the case of creating a slate element that is both
 * command-able and constant. We would likely have no use for an element_id
 * or element_tok since we clearly do not plan to write to this element so
 * why return one?
 *
 * @tparam T The element data type.
 *
 * @param element_path Create the element at this path.
 * @param initial_value The initial value of the element.
 * @param shard Create the element in this shard.
 *
 * @return True on success.
 */
template <typename T>
bool SlateBuilder::create_read_only_element(
    const std::string &element_path, typename slate_info<T>::R initial_value,
    const slate_shard_t shard) {
  slate_element_t element_id = slate_element_default;
  FswAbortIfNot(create_element<T>(element_path, initial_value, shard,
                                  slate_read_only, slate_validator_t(),
                                  element_id),
                false);
  return true;
}

/**
 * Create a private element with a default initial value.
 *
 * @tparam T The element data type.
 * @tparam Access A combination of slate_token_access_t values.
 *
 * @param element_path Create the element at this path.
 * @param shard Create the element in this shard.
 * @param[out] token Returns a token to the element.
 *
 * @return True on success.
 */
template <typename T, int Access>
result_t SlateBuilder::create(const std::string &element_path,
                              const slate_shard_t shard,
                              SlateAccessToken<T, Access> &token) {
  const T initial_value = T();

  FswAbortIfNot(create(element_path, initial_value, shard, slate_private,
                       slate_validator_t(), token),
                false);

  return true;
}

/**
 * Create a private element.
 *
 * @tparam T The element data type.
 * @tparam Access A combination of slate_token_access_t values.
 *
 * @param element_path Create the element at this path.
 * @param initial_value The initial value of the element.
 * @param shard Create the element in this shard.
 * @param[out] token Returns a token to the element.
 *
 * @return True on success.
 */
template <typename T, int Access>
result_t SlateBuilder::create(const std::string &element_path,
                              typename slate_info<T>::R initial_value,
                              const slate_shard_t shard,
                              SlateAccessToken<T, Access> &token) {
  FswAbortIfNot(create(element_path, initial_value, shard, slate_private,
                       slate_validator_t(), token),
                false);

  return true;
}

/**
 * Create an element with a default initial value.
 *
 * @tparam T The element data type.
 * @tparam Access A combination of slate_token_access_t values.
 *
 * @param element_path Create the element at this path.
 * @param shard Create the element in this shard.
 * @param access Access policy to use.
 * @param[out] token Returns a token to the element.
 *
 * @return True on success.
 */
template <typename T, int Access>
result_t SlateBuilder::create(const std::string &element_path,
                              const slate_shard_t shard,
                              const slate_elem_access_t access,
                              SlateAccessToken<T, Access> &token) {
  const T initial_value = T();

  FswAbortIfNot(create(element_path, initial_value, shard, access,
                       slate_validator_t(), token),
                false);

  return true;
}

/**
 * Create an element.
 *
 * @tparam T The element data type.
 * @tparam Access A combination of slate_token_access_t values.
 *
 * @param element_path Create the element at this path.
 * @param initial_value The initial value of the element.
 * @param shard Create the element in this shard.
 * @param access Access policy to use.
 * @param[out] token Returns a token to the element.
 *
 * @return True on success.
 */
template <typename T, int Access>
result_t SlateBuilder::create(const std::string &element_path,
                              typename slate_info<T>::R initial_value,
                              const slate_shard_t shard,
                              const slate_elem_access_t access,
                              SlateAccessToken<T, Access> &token) {
  FswAbortIfNot(create(element_path, initial_value, shard, access,
                       slate_validator_t(), token),
                false);
  return true;
}

/**
 * Create an element with a custom validation function.
 * Note that accesses for elements with size only known at runtime or write
 * accesses for elements with validators are O(log N).
 *
 * @tparam T The element data type.
 * @tparam Access A combination of slate_token_access_t values.
 *
 * @param element_path Create the element at this path.
 * @param initial_value The initial value of the element.
 * @param shard Create the element in this shard.
 * @param access Access policy to use.
 * @param validator The on-write validator to use.
 * @param[out] token Returns a token to the element.
 *
 * @return True on success.
 */
template <typename T, int Access>
result_t SlateBuilder::create(const std::string &element_path,
                              typename slate_info<T>::R initial_value,
                              const slate_shard_t shard,
                              const slate_elem_access_t access,
                              const slate_validator_t &validator,
                              SlateAccessToken<T, Access> &token) {
  static_assert(SlateAccessToken<T, Access>::can_read,
                "We only support tokens that have read access.");

  FswMsgAbortIf(slate_id_is_valid(token.id), false, 100,
                "Cannot create element with provided token! Token is "
                "already bound to an element!");

  slate_element_t element_id = slate_element_default;
  FswAbortIfNot(create_element<T>(element_path, initial_value, shard, access,
                                  validator, element_id),
                false);

  FswAbortIfNot(register_token(element_id, element_path, validator, token),
                false);

  return true;
}

template <class Enum_T, typename T, int Access>
result_t SlateBuilder::create_with_enum(const std::string &element_path,
                                        const slate_shard_t shard,
                                        SlateAccessToken<T, Access> &token) {
  const T initial_value = T();

  FswAbortIfNot(create_with_enum<Enum_T>(element_path, initial_value, shard,
                                         slate_private, slate_validator_t(),
                                         token),
                false);

  return true;
}

template <class Enum_T, typename T, int Access>
result_t SlateBuilder::create_with_enum(const std::string &element_path,
                                        typename slate_info<T>::R initial_value,
                                        const slate_shard_t shard,
                                        SlateAccessToken<T, Access> &token) {
  FswAbortIfNot(create_with_enum<Enum_T>(element_path, initial_value, shard,
                                         slate_private, slate_validator_t(),
                                         token),
                false);

  return true;
}

template <class Enum_T, typename T, int Access>
result_t SlateBuilder::create_with_enum(const std::string &element_path,
                                        const slate_shard_t shard,
                                        const slate_elem_access_t access,
                                        SlateAccessToken<T, Access> &token) {
  const T initial_value = T();

  FswAbortIfNot(create_with_enum<Enum_T>(element_path, initial_value, shard,
                                         access, slate_validator_t(), token),
                false);

  return true;
}

template <class Enum_T, typename T, int Access>
result_t SlateBuilder::create_with_enum(const std::string &element_path,
                                        typename slate_info<T>::R initial_value,
                                        const slate_shard_t shard,
                                        const slate_elem_access_t access,
                                        SlateAccessToken<T, Access> &token) {
  FswAbortIfNot(create_with_enum<Enum_T>(element_path, initial_value, shard,
                                         access, slate_validator_t(), token),
                false);

  return true;
}

template <class Enum_T, typename T, int Access>
result_t SlateBuilder::create_with_enum(const std::string &element_path,
                                        typename slate_info<T>::R initial_value,
                                        const slate_shard_t shard,
                                        const slate_elem_access_t access,
                                        const slate_validator_t &validator,
                                        SlateAccessToken<T, Access> &token) {
  FswAbortIfNot(enum_registry, false);

  /*
   * Create the element.
   */
  FswAbortIfNot(
      create(element_path, initial_value, shard, access, validator, token),
      false);

  /*
   * Register the enum.
   */
  FswAbortIfNot(enum_registry->register_auto_enum<Enum_T>(
                    slate_join_path(subtree_path, element_path)),
                false);

  return true;
}

/**
 * Bind to a slate element. This may only be done before build() is called.
 *
 * @tparam T The element data type.
 * @tparam Access A combination of slate_token_access_t values.
 *
 * @param element_path Bind to the element at this path.
 * @param[out] token Returns a read-write token to the element.
 *
 * @return True on success.
 */
template <typename T, int Access>
result_t SlateBuilder::bind(const std::string &element_path,
                            SlateAccessToken<T, Access> &token) const {
  using Token = SlateAccessToken<T, Access>;
  static_assert(Token::can_read,
                "We only support tokens that have read access.");

  /*
   * You should not be looking up elements at run time.
   */
  FswAbortIf(store->is_built(), false);
  const slate_permission_t permission = store->get_permission();

  if (FswIf(Token::can_read && !slate_can_read(permission))) {
    FswPrefix();
    dbnprintf(500,
              ": Cannot bind to '%s'; reading of existing "
              "elements is not allowed in this Slate.\n",
              element_path.c_str());
    return false;
  }

  if (FswIf(Token::can_write && !slate_can_write(permission))) {
    FswPrefix();
    dbnprintf(500,
              ": Cannot bind to '%s'; writing to existing "
              "elements is not allowed in this Slate.\n",
              element_path.c_str());
    return false;
  }

  FswMsgAbortIf(slate_id_is_valid(token.id), false, 100,
                "Cannot bind element with provided token! Token is "
                "already bound to an element!");

  const std::string full_path = slate_join_path(subtree_path, element_path);
  FswAbortIfNot(require_permitted_path(full_path), false);

  slate_element_t element_id = slate_element_default;

  if (Token::can_write) {
    FswAbortIfNot(store->bind_write(full_path, slate_type_id<T>(), element_id),
                  false);

    /*
     * If a validator is provided, we need a token that does validation.
     * If a token that does validation is provided but there isn't a
     * validation function, we also fail to force callers to use a
     * WriteToken which is more efficient.
     */
    if (FswIf(Token::can_validate != slate_id_has_validator(element_id))) {
      FswPrefix();
      dbnprintf(500,
                ": Cannot bind to '%s'; writing to this element "
                "requires a %s.\n",
                element_path.c_str(),
                Token::can_validate ? "WriteToken" : "WriteValidatorToken");
      return false;
    }
  } else {
    FswAbortIfNot(store->bind_read(full_path, slate_type_id<T>(), element_id),
                  false);
  }

  FswAbortIfNot(slate_token_accountant().register_id(token.id, full_path),
                false);
  token.id = element_id;

  return true;
}

/**
 * Bind to a slate element and register it to an enum type.
 * This may only be done before build() is called.
 *
 * @tparam Enum_T   The enum class to bind the token to.
 * @tparam T        The element data type.
 * @tparam Access   A combination of slate_token_access_t values.
 *
 * @param element_path Bind to the element at this path.
 * @param[out] token Returns a read-write token to the element.
 *
 * @return True on success.
 */
template <class Enum_T, typename T, int Access>
result_t SlateBuilder::bind_with_enum(const std::string &element_path,
                                      SlateAccessToken<T, Access> &token) {
  FswAbortIfNot(enum_registry, false);

  /*
   * Bind the token to the element.
   */
  FswAbortIfNot(bind(element_path, token), false);

  /*
   * Register the enum.
   */
  const std::string full_path = slate_join_path(subtree_path, element_path);
  FswAbortIfNot(enum_registry->register_auto_enum<Enum_T>(full_path), false);

  return true;
}

/**
 * Create a new slate element that uses the memory of the given parent
 * element at the given offset. Bind to this element to access a portion of
 * the parent element, such as a single item in a struct.
 *
 * @param parent_element The name of the parent slate element that we are
 * binding to.
 * @param parent_path SlateBuilder object where the parent_element
 * resides.
 * @param element The name of the slate view element being created.
 * @param element_offset The offset inside the parent element to create this
 *                       view element. The data at this offset _must_ be of
 *                       type T or accessing this element may cause the
 *                       process to crash.
 *
 * @return True on success.
 */
template <typename T>
bool SlateBuilder::create_view(const std::string &parent_element,
                               SlateBuilder parent_path,
                               const std::string &element,
                               size_t element_offset) {
  ReadToken<T> temp_tok;
  FswAbortIfNot(parent_path.bind(parent_element, temp_tok), false);
  FswAbortIfNot(create_view<T>(temp_tok, element, element_offset), false);
  return true;
}

/**
 * Create a new slate element that uses the memory of the given parent
 * element at the given offset. Bind to this element to access a portion of
 * the parent element, such as a single item in a struct.
 *
 * @tparam T The element data type.
 * @tparam ParentT The parent element data type.
 * @tparam ParentAccess The parent element's combination of
 *                      slate_token_access_t values.
 *
 * @param parent_token The parent element's token.
 * @param element_path Create the element at this path.
 * @param element_offset The offset inside the parent element to create this
 *                       view element. The data at this offset _must_ be of
 *                       type T or accessing this element may cause the
 *                       process to crash.
 *
 * @return True on success.
 */
template <typename T, typename ParentT, int ParentAccess>
bool SlateBuilder::create_view(
    SlateAccessToken<ParentT, ParentAccess> &parent_token,
    const std::string &element_path, const size_t element_offset) {
  SlateAccessToken<T, ParentAccess> tok;
  FswAbortIfNot((create_view<T, ParentT, ParentAccess>(
                    parent_token, element_path, element_offset, tok)),
                false);

  return true;
}

/**
 * Create a new slate element that uses the memory of the given parent
 * element at the given offset. Bind to this element to access a portion of
 * the parent element, such as a single item in a struct.
 *
 * @tparam T The element data type.
 * @tparam ParentT The parent element data type.
 * @tparam ParentAccess The parent element's combination of
 *                      slate_token_access_t values.
 *
 * @param parent_token The parent element's token.
 * @param element_path Create the element at this path.
 * @param element_offset The offset inside the parent element to create this
 *                       view element. The data at this offset _must_ be of
 *                       type T or accessing this element may cause the
 *                       process to crash.
 * @param[out] token Returns a token to the view element.
 *
 * @return True on success.
 */
template <typename T, typename ParentT, int ParentAccess>
bool SlateBuilder::create_view(
    SlateAccessToken<ParentT, ParentAccess> &parent_token,
    const std::string &element_path, const size_t element_offset,
    SlateAccessToken<T, ParentAccess> &token) {
  slate_element_t view_element_id = slate_element_default;
  FswAbortIfNot(create_view<T>(parent_token.id, slate_type_id<ParentT>(),
                               element_path, element_offset, view_element_id),
                false);

  FswAbortIfNot(
      register_token(view_element_id, element_path, slate_validator_t(), token),
      false);

  return true;
}

/**
 * Create a new slate element that uses the memory of the given parent
 * element at the given offset. Bind to this element to access a portion of
 * the parent element, such as a single item in a struct.
 *
 * @tparam T The element data type.
 *
 * @param parent_element_id Element ID of the parent.
 * @param parent_type_id Type id of the parent element.
 * @param element_path Create the element at this path.
 * @param element_offset The offset inside the parent element to create this
 *                       view element. The data at this offset _must_ be of
 *                       type T or accessing this element may cause the
 *                       process to crash.
 *
 * @return True on success.
 */
template <typename T>
bool SlateBuilder::create_view(const slate_element_t parent_element_id,
                               const slate_type_t parent_type_id,
                               const std::string &element_path,
                               const size_t element_offset) {
  slate_element_t view_element_id = slate_element_default;
  FswAbortIfNot(create_view<T>(parent_element_id, parent_type_id, element_path,
                               element_offset, view_element_id),
                false);

  return true;
}

/**
 * Create a new slate element that uses the memory of the given parent
 * element at the given offset. Bind to this element to access a portion of
 * the parent element, such as a single item in a struct.
 *
 * @tparam T The element data type.
 *
 * @param parent_element_id Element ID of the parent.
 * @param parent_type_id Type id of the parent element.
 * @param element_path Create the element at this path.
 * @param element_offset The offset inside the parent element to create this
 *                       view element. The data at this offset _must_ be of
 *                       type T or accessing this element may cause the
 *                       process to crash.
 * @param[out] view_element_id Returns the view element ID.
 *
 * @return True on success.
 */
template <typename T>
bool SlateBuilder::create_view(const slate_element_t parent_element_id,
                               const slate_type_t parent_type_id,
                               const std::string &element_path,
                               const size_t element_offset,
                               slate_element_t &view_element_id) {
  /*
   * Verify valid IDs.
   */
  FswAbortIfNot(slate_id_is_valid(parent_element_id), false);

  FswAbortIf(store->is_built(), false);

  const std::string full_path = slate_join_path(subtree_path, element_path);
  FswAbortIfNot(require_permitted_path(full_path), false);

  /*
   * Fail if this is a non-supported slate type.
   */
  FswAbortIfNot(slate_info<T>::is_valid(), false);

  const slate_subsystem_id_t subsystem_id = store->get_subsystem_id();

  /*
   * Request creation by SlateLayout.
   */
  const slate_type_t type_id = slate_type_id<T>();
  const T sizing_value{};
  const size_t value_size = slate_info<T>::size(sizing_value);
  const size_t alignment = slate_info<T>::alignment();
  FswAbortIfNot(store->create_view_element(parent_element_id, parent_type_id,
                                           element_offset, full_path, type_id,
                                           value_size, alignment, subsystem_id,
                                           view_element_id),
                false);

  return true;
}

/**
 * Create an element by ID.
 *
 * @tparam T The element data type.
 *
 * @param element_path Create the element at this path.
 * @param initial_value The initial value of the element.
 * @param shard Create the element in this shard.
 * @param access Access policy to use.
 * @param[out] element_id Returns the element ID.
 *
 * @return True on success.
 */
template <typename T>
bool SlateBuilder::create_element(const std::string &element_path,
                                  typename slate_info<T>::R initial_value,
                                  const slate_shard_t shard,
                                  const slate_elem_access_t access,
                                  slate_element_t &element_id) {
  FswAbortIfNot(create_element<T>(element_path, initial_value, shard, access,
                                  slate_validator_t(), element_id),
                false);

  return true;
}

/**
 * Create an element by ID with a custom validation function.
 * Note that accesses for elements with size only known at runtime or write
 * accesses for elements with validators are O(log N).
 *
 * @tparam T The element data type.
 *
 * @param element_path Create the element at this path.
 * @param initial_value The initial value of the element.
 * @param shard Create the element in this shard.
 * @param access Access policy to use.
 * @param validator The on-write validator to use.
 * @param[out] element_id Returns the element ID.
 *
 * @return True on success.
 */
template <typename T>
bool SlateBuilder::create_element(const std::string &element_path,
                                  typename slate_info<T>::R initial_value,
                                  const slate_shard_t shard,
                                  const slate_elem_access_t access,
                                  const slate_validator_t &validator,
                                  slate_element_t &element_id) {
  FswAbortIf(store->is_built(), false);
  const slate_permission_t permission = store->get_permission();

  if (FswIfNot(slate_can_create(permission, shard))) {
    FswPrefix();
    dbnprintf(500,
              ": Cannot create '%s'; creation of new elements is "
              "not allowed in this slate or shard (%s).\n",
              element_path.c_str(), slate_shard_t_sym.get(shard).c_str());
    return false;
  }

  const std::string full_path = slate_join_path(subtree_path, element_path);
  FswAbortIfNot(require_permitted_path(full_path), false);

  /*
   * Fail if we try to store a non-supported slate type.
   */
  FswAbortIfNot(slate_info<T>::is_valid(), false);

  const slate_subsystem_id_t subsystem_id = store->get_subsystem_id();

  /*
   * Request allocation to SlateLayout.
   */
  element_id = slate_element_default;
  const slate_type_t type_id = slate_type_id<T>();
  const size_t value_size = slate_info<T>::size(initial_value);
  const size_t alignment = slate_info<T>::alignment();
  void *mem = NULL;
  FswAbortIfNot(store->allocate_element(full_path, type_id, value_size,
                                        alignment, shard, access, validator,
                                        subsystem_id, element_id, mem),
                false);
  FswAbortIfNot(mem, false);
  FswAbortIfNot(slate_id_is_valid(element_id), false);

  /*
   * Copy construct the object from the value provided.
   */
  FswAbortIfNot(slate_info<T>::construct(mem, initial_value), false);

  /*
   * If a validator is provided, and it is not a "no-op" validator, check
   * that it is valid.
   */
  if (!validator.is_noop()) {
    /*
     * The validator must be usable. This will be checked again when
     * Slate is built.
     */
    FswAbortIfNot(validator.is_usable(), false);

    /*
     * Check that the validator implementation can be cast to a type
     * that will allow it to be run on this Slate element. This catches
     * cases where a validator of an incorrect type is attached to an
     * element.
     */
    Handle<SlateValidator> validator_impl(validator.internal);

    Handle<SlateTypedValidator<T>> typed_validator;
    FswMsgAbortIfNot(typed_validator.assign_casted(validator_impl), false, 200,
                     "A validator of an incompatible type was attached "
                     "to element \"%s\".",
                     full_path.c_str());

    /*
     * Run the validator on the provided initial value, and make sure
     * that it succeeds.
     */
    typename slate_info<T>::W value = slate_info<T>::from_mem(mem);

    FswAbortIfNot(typed_validator->validate(initial_value, value), false);
  }

  return true;
}

/**
 * Get the ID of a Slate element.
 *
 * @tparam T The element data type.
 *
 * @param element_path Get the element at this path.
 * @param[out] element_id Returns the element ID.
 *
 * @return True on success.
 */
template <typename T>
bool SlateBuilder::get_element_id(const std::string_view element_path,
                                  slate_element_t &element_id) const {
  const slate_type_t type_id = slate_type_id<T>();
  FswAbortIfNot(get_element_id(element_path, type_id, element_id), false);

  return true;
}

/**
 * Returns true if an element with the given path and type exists.
 *
 * @tparam T The element data type.
 *
 * @param element_path Check for an element at this path.
 *
 * @return True if path exists and type matches.
 */
template <typename T>
bool SlateBuilder::element_exists(const std::string &element_path) const {
  const slate_type_t type_id = slate_type_id<T>();
  return element_exists(element_path, type_id);
}

/**
 * Returns the initial value provided to create() when the passed token was
 * created. Only valid during the build phase; at runtime, use
 * `Slate::load()`.
 *
 * @tparam T The element data type.
 *
 * @param token Token whose initial value to query.
 * @param value[out] On return, the initial value of `token`.
 *
 * @return True on success. False if the Slate has been built or if `token`
 *         has not been created or bound.
 */
template <typename T>
bool SlateBuilder::get_initial_value(const SlateToken<T> &token,
                                     typename slate_info<T>::O value) const {
  FswAbortIfNot(get_element_initial_value<T>(token.id, value), false);
  return true;
}

/**
 * Returns the initial value provided to create() when the passed element
 * was created. Only valid during the build phase; at runtime, use
 * `Slate::load_r()`, `Slate::load_rw()` or `Slate::load_rwv()` as
 * appropriate.
 *
 * @tparam T The element data type.
 *
 * @param elem_id ID of the element whose initial value to query.
 * @param value[out] On return, the initial value of `elem_id`.
 *
 * @return True on success. False if the Slate has been built or if
 *         `elem_id` is invalid.
 */
template <typename T>
bool SlateBuilder::get_element_initial_value(
    const slate_element_t elem_id, typename slate_info<T>::O value) const {
  FswAbortIf(store->is_built(), false);

  const slate_type_t type_id = slate_info<T>::type_id();
  B2c data;
  FswAbortIfNot(layout.get_element_initial_memory(elem_id, type_id, data),
                false);
  FswAbortIfNot(data.buf(), false);
  FswAbortIfNot(data.len(), false);

  value = slate_info<T>::from_mem(data.buf());

  return true;
}

/**
 * Gets the full path of a slate element by token. The path is
 * relative to the root slate, even if this is a sub_slate.
 * If the element is an alias (view of an element of the same type), it will
 * return the original element path.
 *
 * @tparam T The element data type.
 *
 * @param token Token for the element to query.
 * @param[out] path On return, the path of the element.
 *
 * @return True on success.
 */
template <typename T>
bool SlateBuilder::get_path(const SlateToken<T> &token,
                            std::string &path) const {
  return get_first_element_path(token.id, slate_type_id<T>(), path);
}

/**
 * Registers an enum for a Slate element using its token.
 * An EnumRegistry must be added to the SlateBuilder, either directly,
 * or via inheritance of a parent SlateBuilder's EnumRegistry
 *
 * @param token The token for the element.
 * @param enum_name The name of the enum.
 * @param symbol_table The SymbolTable that describes the enum.
 * @param strip_prefix If this is a prefix in the enumerated values,
 *                     strip it away. Useful for making auto_enum
 *                     SymbolTable's more readable.
 * @return True on success.
 */
template <typename T>
bool SlateBuilder::register_enum(const SlateToken<T> &token,
                                 const std::string &enum_name,
                                 const SymbolTable &symbol_table,
                                 const std::string &strip_prefix) {
  std::string path;
  FswAbortIfNot(get_path(token, path), false);
  std::string relative_path;
  FswAbortIfNot(get_relative_path(path, relative_path), false);
  FswAbortIfNot(
      register_enum(relative_path, enum_name, symbol_table, strip_prefix),
      false);
  return true;
}

} /* end namespace Drone */

#endif /* SLATE_BUILDER_H */