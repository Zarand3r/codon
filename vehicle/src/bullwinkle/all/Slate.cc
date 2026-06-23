/**
 * @author Richard Bao
 * @date   01/28/2025
 */

#include "src/bullwinkle/all/Slate.h"

#include "src/hash/xxh.h"

namespace Drone
{
     /**
      * Default constructor
      * 
      * Slates constructed this way are not valid - assign a valid Slate
      * to this one (e.g. the one from a SlateBuilder) before use.
      */
     Slate::Slate(): memory(), last_slate_default_token_count()
     {}

     /**
      * Constructor.
      * 
      * @param _memory The actual SlateMemory instance holding data.
      */
     Slate::Slate(Handle<SlateMemory> _memory):
          memory(_memory),
          last_slate_default_token_count(0)
     {
          FswAssert(memory);
     }

     /**
      * Copy constructor. 
      * 
      * @param slate The slate we are being copied from. 
      */
     Slate::Slate(const Slate &slate):
          memory(slate.memory),
          last_slate_default_token_count(slate.last_slate_default_token_count)
     {}

     /**
      * Assignment operator.
      * 
      * @param slate The slate we are being copied from.
      * 
      * @return A copy of slate.
      */
     Slate Slate::operator=(const Slate &slate)
     {
          memory = slate.memory;
          last_slate_default_token_count = slate.last_slate_default_token_count;
          return *this;
     }

     /**
      * Roll a new frame. No memory references to elements inside the
      * Slate are valid after this point. 
      * 
      * @return True on success.alignas
      */
     bool Slate::roll_frame()
     {
          FswAbortIfNot(memory, false);
          FswAbortIfNot(memory->roll_frame(), false);
          
          /**
           * Complain if unbound tokens have suddenly appeared. 
           */
          if (_slate_deafult_token_count > last_slate_default_token_count)
          {
               FswPrefix();
               dbnprintf(200,
                         ": WARNING %zu leaked token(s) detected!\n",
                         _slate_default_token_count);
          }

          last_slate_default_token_count = _slate_default_token_count;
          return true;
     }

     /**
      * Return the raw memory block for an entire shard.
      * 
      * @param      shard Return the memory for this shard.
      * @param[out] mem Returns the raw memory block.
      * 
      * @return True on success.
      */
     bool Slate::get_shard_memory(const slate_shard_t shard, B2c &mem) const
     {
          FswAbortIfNot(memory, false);
          FswAbortIfNot(memory->get_shard_memory(shard, mem), false);
          return true;
     }

     /**
      * Return the raw memory block for an entire shard as a writeable buffer.
      * 
      * @param      shard Return the memory for this shard.
      * @param[out] mem Returns the raw memory block.
      * 
      * @return True on success.
      */
     bool Slate::get_shard_memory(const slate_shard_t shard, B2 &mem)
     {
          FswAbortIfNot(memory, false);
          FswAbortIfNot(memory->get_shard_memory(shard, mem), false);
          return true;
     }

     /**
      * Set the raw memory block for an entire shard.
      * 
      * @param shard Set the memory for this shard.
      * @param mem The raw memory block.
      * 
      * @return True on success.
      */
     bool Slate::set_shard_memory(const slate_shard_t shard, const B2c &mem)
     {
          FswAbortIfNot(memory, false);
          FswAbortIfNot(memory->set_shard_memory(shard, mem), false);
          return true;
     }

     /**
      * Compute the 64-bit hash of a Slate shard. 
      * 
      * @param      shard Compute the hash of this shard.
      * @param[out] shard_hash Returns the hash for this shard. 
      * 
      * @return True on success.
      */
     bool
     Slate::compute_hash(const slate_shard_t shard, UINT64 &shard_hash) const
     {
          FswAbortIfNot(memory, false);
          
          if (memory->is_shard_empty(shard))
          {
               shard_hash = 0;
               return true;
          }

          B2c shard_memory;
          Hash128 shard_layout_hash;
          FswAbortIfNot(memory->get_shard_memory(shard, shard_memory), false);
          FswAbortIfNot(memory->get_shard_layout_hash(shard, shard_layout_hash),
                        false);

          FswAbortIfNot(shard_memory.buf(), false);
          const Hash128 hash = digest_xxh128(shard_memory.buf(),
                                             shard_memory.len(),
                                             shard_layout_hash /* seed */);
          shard_hash = hash.u64[0] ^ hash.u64[1];
          return true;
     }

     /**
      * Compute the deltas between two memory blocks using the metadata
      * for the given shard.
      * 
      * @param      shard Use this shard's metadata as the layout for
      *                   #mem1 and #mem2
      * @param      mem1 First copy of the shard data.
      * @param      mem2 Second copy of the shard data.
      * @param[out] deltas Returns the deltas, if any. 
      * @param      max_deltas Return at most this number of deltas.
      *                        -1 to disable the limit.
      * 
      * @return False if deltas could not be computed, e.g. if the memory
      *         sizes don't match the shard metadata.
      */
     bool Slate::compute_deltas(const slate_shard_t shard,
                                const B2c &mem1,
                                const B2c &mem2,
                                shard_delta_v &deltas,
                                const int max_deltas) const
     {
          FswAbortIfNot(memory, false);
          if (memory->is_shard_empty(shard) && mem1.len() == 0 && mem2.len() == 0)
          {
               /**
                * Similarly to SlateMemory::compute_shard_deltas(), return false
                * if deltas array is not empty.
                */
               FswAbortIfNot(deltas.empty(), false);
               return true;
          }
          
          FswAbortIfNot(
               memory->compute_shard_deltas(shard, mem1, mem2, deltas, max_deltas),
               false);

          return true;

     }

     /**
      * Return true if the underlying slate has been built, or false if it has
      * not yet been built. 
      * 
      * @return True if built. 
      */
     bool Slate::is_built() const
     {
          return memory->is_built();
     }
} /* end namespace Drone */