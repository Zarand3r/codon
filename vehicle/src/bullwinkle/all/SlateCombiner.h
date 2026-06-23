/**
 * @author Richard Bao
 * @date   01/28/2025
 */

#ifndef SLATE_COMBINER_H
#define SLATE_COMBINER_H

#include "src/bullwinkle/all/Configs.h"
#include "src/bullwinkle/all/Signal.h"
#include "src/bullwinkle/all/Slate.h"
#include "src/bullwinkle/all/SlateBuilder.h"
#include "src/bullwinkle/all/SlateSharer.h"
#include "src/bullwinkle/all/config_file.h"
#include "src/bullwinkle/all/core/fsw.h"
#include "src/bullwinkle/all/core/drone_types.h"
#include "src/bullwinkle/all/core/stl_util.h"
#include "src/bullwinkle/all/core/util.h"
#include "src/bullwinkle/all/runtime.h"
#include "src/bullwinkle/all/static_vector.h"

#include <string>
#include <vector>

namespace Drone
{
    /**
     * Describes a component of input into the SlateCombiner.
     */
    struct slate_combiner_in_t
    {
        slate_combiner_in_t();
        slate_combiner_in_t(SlateBuilder _builder, const std::string &_name);

        /**
         * The builder to use.
         */
        SlateBuilder builder;

        /**
         * The name of this source. Determines the slate names of
         * various per-source metrics.
         */
        std::string name;
    };
    typedef std::vector<slate_combiner_in_t> slate_combiner_in_v;

    /**
     * This class reads a configuration file with a list of slate elements
     * to combine. It will read from three source slates and combine them
     * into a single destination slate.
     *
     * Each source slate has a defined "freshness" element that is required to
     * change before that slate's values will be used for the median
     * calculation. By default, on every cycle, if the "freshness" element has
     * not been updated, the source is "stale" and will not be used for
     * combination.
     *
     * The staleness threshold can be increased to longer than one cycle to
     * allow combining inputs in non-time-synchronized contexts, but to avoid
     * unpredictable behavior, this should only be used when the input values
     * are idempotent, and change slowly.
     *
     * The configuration file looks like this:
     *
     * <element 1 path> <element 1 type>
     * <element 2 path> <element 2 type>
     * ...
     * <element n path> <element n type>
     *
     * The output elements are a either a 3/3, 2/2, 1/1 median of the source
     * Slate values on an element-by-element basis depending on freshness, or a
     * direct copy of all the values from the first fresh source. These results
     * are writen to the destination slate. If no inputs are fresh then no
     * outputs will be updated.
     *
     * Only a basic set of types is supported. See \ref init() for details.
     */
    class SlateCombiner: public SignalHandler
    {
    public:
        SlateCombiner();

        bool init(const Configs &configs,
                  const std::string &fresh,
                  const slate_combiner_in_v &builders_in,
                  SlateBuilder builder_out,
                  const std::string &config_key,
                  const bool bind_outputs = false,
                  const std::string &combiner_name = "combiner");

        bool init(const Configs &configs,
                  const std::string &fresh,
                  const slate_combiner_in_v &builders_in,
                  SlateBuilder builder_out,
                  const sharer_config_v &config_list,
                  const bool bind_outputs = false,
                  const std::string &combiner_name = "combiner",
                  const slate_elem_access_t access = slate_read_only,
                  const bool _downselect = false,
                  const bool bind_metrics = false,
                  const UINT8 _stale_threshold = 1U);

        static bool register_output_enum(const slate_combiner_in_v &builders_in,
                                         SlateBuilder builder_out,
                                         const std::string &path,
                                         const bool bind_outputs);

        bool combine() RUNTIME;
        bool
        validate_and_combine(const nano_t accepted_sequence_number) RUNTIME;

        /**
         * To enable some optimizations, we specify a maximum number of
         * allowed sources.
         */
        static const size_t num_sources_max = 3;

        /**
         * Holds freshness metrics for each source.
         */
        struct source_t
        {
            source_t(): fresh_tok(), fresh_last_tok(), connected_tok()
            {}

            bool init(const SlateBuilder &source_builder,
                      const std::string &source_name,
                      const std::string &fresh,
                      const UINT8 _stale_threshold,
                      SlateBuilder &combiner_builder);

            /**
             * The source freshness element we monitor.
             */
            ReadToken<INT64> fresh_tok;

            /**
             * The last freshness we read.
             */
            WriteToken<INT64> fresh_last_tok;

            /**
             * The number of cycles since the freshness element was updated.
             * Capped at stale_threshold.
             */
            WriteToken<UINT8> fresh_age_tok;

            /**
             * The current status of the freshness.
             */
            WriteToken<bool> connected_tok;
        };

    private:
        typedef std::vector<slate_element_t> slate_element_v;

        /**
         * A bin of elements along with a processing function. We keep
         * one of these per unique (type, operation) tuple.
         */
        struct element_bin_t
        {
            element_bin_t();

            /**
             * All input IDs to process. The IDs are in batches of the
             * number of sources (N) plus one (e.g. if 3 sources, (0 1 2
             * 3), (4 5 6 7), (8 9 10 11)). The first one is the output
             * element, the last N are inputs. They are all in the same
             * linear array to improve cache coherency and memory
             * bandwidth usage while doing scans.
             */
            slate_element_v ids;

            /**
             * Typedefs for pointers to the operation functions.
             *
             * @{
             */
            typedef bool (SlateCombiner::*eq_fn_t)(size_t src1,
                                                   size_t src2,
                                                   const element_bin_t &bin,
                                                   bool &result) RUNTIME;

            typedef bool (SlateCombiner::*copy_fn_t)(
                const size_t index,
                const element_bin_t &bin) RUNTIME;

            typedef bool (SlateCombiner::*max_fn_t)(
                const size_t index1,
                const size_t index2,
                const element_bin_t &bin) RUNTIME;

            typedef bool (SlateCombiner::*median_fn_t)(
                const element_bin_t &bin) RUNTIME;
            /**
             * @{
             */

            /**
             * Operation function pointers for each operation.
             *
             * @{
             */
            eq_fn_t eq_fn;
            copy_fn_t copy_fn;
            max_fn_t max_fn;
            median_fn_t median_fn;
            /**
             * @}
             */
        };

        template <typename T>
        bool init_element(const slate_combiner_in_v &builders_in,
                          SlateBuilder builder_out,
                          const std::string &path,
                          const bool bind_outputs,
                          element_bin_t &bin,
                          const slate_elem_access_t access);

        template <typename T>
        bool bin_eq(size_t src1,
                    size_t src2,
                    const element_bin_t &bin,
                    bool &result) RUNTIME;

        template <typename T>
        bool bin_copy(const size_t src, const element_bin_t &bin) RUNTIME;

        template <typename T>
        bool bin_max(const size_t src1,
                     const size_t src2,
                     const element_bin_t &bin) RUNTIME;

        template <typename T>
        bool bin_median(const element_bin_t &bin) RUNTIME;

        /**
         * Is the object initialized?
         */
        bool is_init;

        /**
         * Whether to use "downselect" mode when combining instead of the
         * default per-element median. This can be used to avoid "tearing"
         * messages that require inter-element coherency. If there is not a
         * clear majority (two or more exactly equal sources), then the first
         * connected source will always be used.
         */
        bool downselect;

        /**
         * The number of sources we're configured with.
         */
        size_t num_sources;

        /**
         * A "fresh" source's inputs will be considered "stale" (i.e., the
         * source is disconnected) after this many cycles without an update.
         *
         * The standard value for time-synchronized use cases is one cycle; this
         * can be configured to allow non-time-synchronized processes to be
         * resilient to "tearing" between inputs from multiple sources.
         */
        UINT8 stale_threshold;

        /**
         * The list of element bins.
         */
        std::vector<element_bin_t> bins;

        /**
         * The list of sources.
         */
        static_vector<source_t, num_sources_max> sources;

        /**
         * The combined connected flag that indicates whether any source is
         * connected.
         */
        WriteToken<bool> combined_connected_tok;

        /**
         * The run-time slate.
         */
        INFRASTRUCTURE(Slate) slate;
    };

} /* end namespace Drone */

#endif /* SLATE_COMBINER_H */