// Vector implementation -*- C++ -*-

// Copyright (C) 2001-2024 Free Software Foundation, Inc.
//
// This file is part of the GNU ISO C++ Library.  This library is free
// software; you can redistribute it and/or modify it under the
// terms of the GNU General Public License as published by the
// Free Software Foundation; either version 3, or (at your option)
// any later version.

// This library is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.

// Under Section 7 of GPL version 3, you are granted additional
// permissions described in the GCC Runtime Library Exception, version
// 3.1, as published by the Free Software Foundation.

// You should have received a copy of the GNU General Public License and
// a copy of the GCC Runtime Library Exception along with this program;
// see the files COPYING3 and COPYING.RUNTIME respectively.  If not, see
// <http://www.gnu.org/licenses/>.

/*
 *
 * Copyright (c) 1994
 * Hewlett-Packard Company
 *
 * Permission to use, copy, modify, distribute and sell this software
 * and its documentation for any purpose is hereby granted without fee,
 * provided that the above copyright notice appear in all copies and
 * that both that copyright notice and this permission notice appear
 * in supporting documentation.  Hewlett-Packard Company makes no
 * representations about the suitability of this software for any
 * purpose.  It is provided "as is" without express or implied warranty.
 *
 *
 * Copyright (c) 1996
 * Silicon Graphics Computer Systems, Inc.
 *
 * Permission to use, copy, modify, distribute and sell this software
 * and its documentation for any purpose is hereby granted without fee,
 * provided that the above copyright notice appear in all copies and
 * that both that copyright notice and this permission notice appear
 * in supporting documentation.  Silicon Graphics makes no
 * representations about the suitability of this  software for any
 * purpose.  It is provided "as is" without express or implied warranty.
 */

/** @file stl/coupled_vectors.h
 *  This is an internal header file, included by other library headers.
 *  Do not attempt to use it directly. 
 *  @headername{coupled_vectors.h}
 */

#ifndef _COUPLED_VECTORS_H
#define _COUPLED_VECTORS_H

#include <array>
#include <type_traits>
#include <memory>
#include <cstring> // std::memcpy
#include <cstddef> // std::byte
#include <tuple>
#include <vector>
#include <algorithm> // std::transform

// __cplusplus >= 17
namespace stl
{
  /**
   * Vector of Structs or Struct of Vectors?
   * 
   * take the example of a pixel structure:
   * Is it more efficient to write as:
   *    std::vector<uint_32> RGBA{Size};
   * 
   * or split it to 04 buffers as:
   *    std::vector<uint_8> R{Size}; 
   *    std::vector<uint_8> G{Size}; 
   *    std::vector<uint_8> B{Size}; 
   *    std::vector<uint_8> A{Size};
   * 
   * While the first choice loads into cacheline the whole pixel data;
   * it is not effecient to manipulate one channel (which is very common in 
   * image manipulation softwares), and we are obliged to use bit manipulation
   * to extract that channel; or use strides to gather the desired channel
   * directly from cacheline.
   * The second choice gives us more flexibilty, and open the door for
   * Vectorization and Parallelism technics.
   * 
   *   The need for a container to allow us to manipulate many buffers has its
   * advantages. The physically separated buffers are logically coupled by
   * operations and semantics (all buffers grow and shrink by the same amount,
   * all buffers together represent a logical idea e.g: a pixel)
   * 
   * So, instead of creating many std::vectors to manage these buffers, and
   * synchronize them for any container operation, it is more effecient to keep
   * the management in one structure.
   *
   * ENTER
   *    coupled_vectors
   *  A ;vector-like; allocator aware container to create, manipulate and process
   * many dynamic memory buffers; of different value types; at the same time.
   *   
   * The model is based and how GPU's work: 
   *    Many dissociated buffers vs One global threaded operation.
   * 
   *  The challenge here is in the allocator. 
   * There are two allocation strategies:
   *    1- all buffers contiguously in the same memory arean.
   *    2- allocate each buffer individaully.
   * 
   *  The first strategy is subject to the memory arena size that the underlying
   * system can provide. If the combined size in bytes:
   * n_bytes = number of types * sum(sizeof(types)...)
   * exceeds what the underlying platform can provide, then after a cetrain 
   * threashold we will run out of contiguous memory.
   * 
   *  The second strategy is subject to the alignment requirement of each 
   * involved value type. Since the allocator has only one type parameter 
   * (i.e allocator<typename _Tp>; one allocator specialization for each type)
   * this will conflicts with how to allocate many buffers of different 
   * value types, without duplicating the allocator instance, or keep
   * changing it by copy-assignement, especially if the allocator is stateful.
   * 
   *   The solution, opted-for in this implementation, is to decay the allocator
   * (i.e rebind it) the an allocator of type:
   * allocator<byte-like-type>
   * the 'byte-like-type' object type must respect the following constraints:
   *  - sizeof(byte-like-type) == 1
   *  - alignof(byte-like-type) is tunnable
   *  - std::is_same_v<byte-like-type(align_1), byte-like-type(align_2)> == false
   *
   * The idea here is to unify allocation for all different types, and respect
   * each type alignment requirement (even extended-alignment types).
   * Failing to do so, will sanction the allocation operation with a lot of
   * wasted memory caused by padding to reach correct alignment.
   * 
   * On the other hand, pointers returned by the allocators are byte-like pointers,
   * which can be staticaly casted back to the appropriate pointer type, since
   * 'char*' can be aliased to any pointer type.
   * 
  */

  // primary class template
  template<typename _Tp, typename _Alloc = std::allocator<_Tp>>
  class coupled_vectors : public std::vector<_Tp, _Alloc>
  { };

  // partial specialization
  // std::pair is a special case of std::tuple
  template<typename..._Ts, typename _Alloc>
  class coupled_vectors<std::tuple<_Ts...>, _Alloc>;

  namespace __detail
  {    
    /// @brief _Ptrs_array_base: A base class storing pointers to the first byte
    ///        to several buffers.
    ///        The idea here, is to store pointers to buffers of different types,
    ///        and since the std::array accepts only one type as a typed template
    ///        parameter, then we will store all pointers as void pointers.
    ///        Later on when we need to access the elements of the buffers,
    ///        we static cast back to the original data type.
    /// @tparam N : an unsigned integer representing the number of coupled
    ///         bufferss.
    template<std::size_t _Nm>
    struct _Ptrs_array_base :  std::array<void*, _Nm>
    {
      using  _void_ptr = void*;
      using _base_type = std::array<_void_ptr, _Nm>;

      using _base_type::_base_type;

      template<typename _Ptr_t>
      constexpr 
      _Ptrs_array_base(std::array<_Ptr_t, _Nm> const& __arr)
      {
        static_assert(std::is_pointer_v<_Ptr_t>
                    , "all args must be of pointer-type to data store.");
        std::memcpy(this->data(), __arr.data(), _Nm);
      }

      // Conversion from any pointers type.
      // N.B : The information about types is lost by static casting;
      //       Make sure that is stored somewhere.
      template<typename... _Ptr_t>
      constexpr _Ptrs_array_base(_Ptr_t... __bufs)
      : _base_type{static_cast<_void_ptr>(__bufs) ...}
      { 
        static_assert(sizeof...(_Ptr_t) != _Nm
                    , "number of buffers must be equal"
                      "to the predefined size of this struct");
        static_assert(std::is_pointer_v<_Ptr_t>&& ...
                    , "all args must be of pointer-type to data store.");
      }      
    }; // _Ptrs_array_base
    
    // helper struct to customize alignement while keeping the size equal to 1
    template<std::size_t _N>
    struct aligned
    {
      enum struct alignas(1)   Al_1   : unsigned char{};
      enum struct alignas(2)   Al_2   : unsigned char{};
      enum struct alignas(4)   Al_4   : unsigned char{};
      enum struct alignas(8)   Al_8   : unsigned char{};
      enum struct alignas(16)  Al_16  : unsigned char{};
      enum struct alignas(32)  Al_32  : unsigned char{};
      enum struct alignas(64)  Al_64  : unsigned char{};
      enum struct alignas(128) Al_128 : unsigned char{};
      enum struct alignas(256) Al_256 : unsigned char{};

      using Al_tuple = 
      std::tuple<Al_1, Al_2, Al_4, Al_8, Al_16, Al_32, Al_64, Al_128, Al_256>;

      // Compile time log2 function.
      // helper to calculate the index into the tuple.
      template<std::size_t _M = _N, std::size_t _Idx = 0>
      constexpr std::size_t log2()
      {
        if constexpr(_M)
          return log2<(_M>>1), (_Idx + 1)>();
        else
          return (_Idx - 1) < 0 ? 0UL : (_Idx-1);
      }

      using byte = typename std::tuple_element<log2<>(), Al_tuple>::type;
    };

    // helper type aliases
    template<typename _Ap>
    using __Value_type = typename std::allocator_traits<_Ap>::value_type;

    template<std::size_t _AlignOf>
    using _aligned_byte = typename aligned<_AlignOf>::byte;

    // helper alias type
    template<typename _Alloc, std::size_t _AlignOf>
    using _rebind_Allocator = 
    typename std::allocator_traits<_Alloc>::rebind_alloc<_aligned_byte<_AlignOf>>;    

    /// @brief _Alloc_base: base class to manage allocation of the coupled
    ///        buffers.
    ///        Two strategies could be used; either allocate a contiguous arena
    ///        to hold all buffers, Or allocate the necessary size for each
    ///        buffer separatly. The minimum size to switch from arena to
    ///        individual buffers, needs to be mesured for effeciency.
    ///        When using arena strategy, the buffer allocated must be aligned
    ///        at the first type boundary .i.e (alignas(_Tp0)).
    /// @tparam _Alloc: allocator type. 
    ///         For Arena (_Use_arena = true) allocation strategy, we rebind
    ///         this allocator to std::byte type, to reserve a contiguous chunck
    ///         of memory, then we assign each coupled buffer its appropriate
    ///         partition of memory from this arena (type alignment 
    ///         and padding must be respected between each buffer's partition
    ///         when calculating each partition size and begin pointer.).
    ///         For Individual buffers allocation (_Use_arena = false), we
    ///         allocate each buffer array separately, respecting alignment.
    /// @tparam _Use_arena: bool value. True for allocating an Arena where
    ///         all buffers live together. False for individual allocations
    ///         for each buffer.
    
    // primary base class
    template<typename _Allocator, bool _Use_arena>
    struct _Alloc_base;

    // Arena allocation strategy
    // the first byte of the allocated buffer should be aligned
    // at 1st type boundaries.
    template<typename _Alloc>
    struct _Alloc_base<_Alloc, true>
    : public _rebind_Allocator<_Alloc, alignof(__Value_type<_Alloc>)> 
    {
      static_assert(0 < alignof(__Value_type<_Alloc>) 
            && alignof(__Value_type<_Alloc>) <= 256, "Unsupported Alignement");
      
      using _base_type = _rebind_Allocator<_Alloc, alignof(__Value_type<_Alloc>)>
      using _alloc_traits = std::allocator_traits<_base_type>;
      using value_type = typename _alloc_traits::value_type;
      using pointer = typename _alloc_traits::pointer;
      using size_type = typename _alloc_traits::size_type;

      // base type import all Ctors
      using _base_type::_base_type;

      [[nodiscard]]
      _Alloc_base& 
      _M_get_allocator() noexcept
      { return *this; }

      [[nodiscrad]]
      const _Alloc_base& 
      _M_get_allocator() const noexcept
      { return *this; }

      template<typename _Tail>
      [[nodiscard]]
      constexpr size_type
      _M_byte_size(size_type __last_offset, size_type __n_elem) const
      {
        return __last_offset + (__n_elem * sizeof(_Tail)); 
      }

      template<typename..._Ts>
      [[nodiscard]] 
      constexpr auto 
      _M_nbytes_and_offsets(size_type _n_elem) const ->
            std::tuple<size_type, std::array<size_type, sizeof...(_Ts)>>
      {       
        constexpr std::size_t _N = sizeof...(_Ts);
        static_assert(_N != 0, "At least one type must be provided!");

        constexpr std::array<size_type, _N> _Szof = {sizeof(_Ts)...};
        constexpr std::array<size_type, _N> _Algnof = {alignof(_Ts)...};

        // calculate total size in std::byte
        std::array<size_type, _N> _M_diffs{0};

        for(size_type it = 1; it < _N; ++it)
        {
          auto v = _Szof[it-1] * _n_elem;
          auto cum = v + _M_diffs[it-1];
          if(cum % _Algnof[it] == 0)
            _M_diffs[it] = cum;
          else
            _M_diffs[it] = cum + (_Algnof[it] - (cum % _Algnof[it]));
        }
        // tail type
        using _Tail = 
          typename std::tuple_element<_N - 1, std::tuple<_Ts...>>::type;

        const size_type _n_bytes = _M_byte_size<_Tail>(_M_diffs.back(), _n_elem);
        return {_n_bytes, _M_diffs};
      }

      template<typename..._Ts>
      struct _M_allocate_hlpr
      {
        [[nodiscard]]
        constexpr pointer 
        operator()(size_type _n_elem) const
        {
          auto [_n_bytes, _M_diffs] = _M_nbytes_and_offsets<_Ts...>(_n_elem);
          return  _alloc_traits::allocate(_M_get_allocator(), _n_bytes);
        }
      };

      template<typename..._Ts>
      struct _M_allocate_hlpr<std::tuple<_Ts...>>
      {
        [[nodiscard]]
        constexpr pointer 
        operator()(size_type _n_elem) const
        {
          auto [_n_bytes, _M_diffs] = _M_nbytes_and_offsets<_Ts...>(_n_elem);
          return  _alloc_traits::allocate(_M_get_allocator(), _n_bytes);
        }
      };

      template<typename _Tuple>
      [[nodiscard]]
      constexpr pointer
      _M_allocate(size_type _n_elem)
      {
        return _M_allocate_hlpr<_Tuple>{}(_n_elem);
      }

      template<typename..._Ts>
      struct _M_deallocate_hlpr
      {
        void operator()(pointer _ptr, size_type _n_elem) const
        {
          auto [_n_bytes, _M_diffs] = 
            _M_nbytes_and_offsets<_Ts...>(_n_elem);
          return _alloc_traits::deallocate(_M_get_allocator(),_ptr, _n_bytes);~
        }
      };

      template<typename..._Ts>
      struct _M_deallocate_hlpr<std::tuple<_Ts...>>
      {
        void operator()(pointer _ptr, size_type _n_elem) const
        {
          auto [_n_bytes, _M_diffs] = 
            _M_nbytes_and_offsets<_Ts...>(_n_elem);
          return _alloc_traits::deallocate(_M_get_allocator(), _ptr, _n_bytes);
        }
      };

      template<typename _Tuple>
      void
      _M_deallocate(pointer _ptr, size_type _n_elem)
        noexcept(noexcept(_alloc_traits::deallocate({}, {}, {})))
      {
        return _M_deallocate_hlpr<_Tuple>{}(_ptr, _n_elem);
      }

      
    };

    //  Partial specialization for individual allocation strategy
    // using void pointer alignement as the default allocation alignement 
    // for one byte, since it most likely will suite all alignement needs.
    // for extended aligmenet see comments below.
    template<typename _Alloc>
    struct _Alloc_base<_Alloc, false> 
    : public _rebind_Allocator<_Alloc, alignof(void*)>
    {
      using _byte_type = aligned_byte<alignof(void*)>;
      using _base_type = _rebind_Allocator<_Alloc, alignof(void*)>;
      using _alloc_traits = std::allocator_traits<_base_type>;
      using pointer = typename _alloc_traits::pointer;
      using size_type = typename _alloc_traits::size_type;

      [[nodiscard]]
      _Alloc_base& 
      _M_get_allocator() noexcept
      { return *this; }

      [[nodiscard]]
      const _Alloc_base& 
      _M_get_allocator() const noexcept
      { return *this; }
      
      // Assumption: alignof(_Tp) <= 256.
      //  For alignement greater than alignof(void*), we readjust the
      // first byte address pointer to the desired alignment and we
      // store the offset to the original pointer, in the One byte
      // address, before the new pointer.
      template<typename _Tp>
      [[nodiscard]] 
      constexpr pointer 
      _M_allocate(size_type _n_elem)
      {
        static_assert(0 < alignof(_Tp) && alignof(_Tp) <= 256
                    , "Unsupported Alignement");

        constexpr std::size_t _M_align_val = alignof(_byte_type);
        if constexpr (alignof(_Tp) <= _M_align_val)
        {
          constexpr size_type _n_bytes = _n_elem * sizeof(_Tp);
          pointer _M_ptr = _alloc_traits::allocate(*this, _n_bytes);
          return _M_ptr;
        }
          
        //  For any other specific alignement, we need to re-adjust the
        // request byte size, and keep track of the old pointer
        else 
        {
          constexpr size_type _Align_diff = alignof(_Tp) - _M_align_val;
          size_type _n_bytes = _n_elem * sizeof(_Tp) + _Align_diff;
          const auto _old_n = _n_bytes;
          
          pointer _M_ptr = _alloc_traits::allocate(_M_get_allocator(), _n_bytes);
          // failed allocation
          if(_M_ptr == pointer{})
            return pointer{};

          // adjust pointer according to alignement
          const pointer _M_old_ptr = _M_ptr;
          if(not std::align(alignof(_Tp), sizeof(_Tp)
                      , static_cast<void*&>(_M_ptr), _n_bytes))
          {
            _alloc_traits::deallocate(_M_get_allocator(), _M_ptr, _n_bytes);
            // keep the behavior of the base allocator class
            // if it throws then throw.
            if constexpr (noexcept(_alloc_traits::allocate({},{})))
              return pointer();
            else
              throw std::bad_alloc();
          }
          // For successfull realignement, store offset from new pointer.
          //  N.B: An extended alignement of a value more than 256 is 
          // unlikely to be used, thus it is acceptable to reserve one byte
          // to store the offset.
          // Offset values to store in range ]0, (256-8)];
          const ptrdiff_t _M_offset = _M_ptr - _M_old_ptr;
          if(_M_offset != 0 )
            _alloc_traits::construct(_M_get_allocator(), (_M_ptr - 1)
                                   , static_cast<unsigned char>(_M_offset));
          else // move forward the new pointer
            _alloc_traits::construct(_M_get_allocator(), 
            ((_M_ptr+=_Align_diff) - 1), static_cast<unsigned char>(_M_offset));

          return _M_ptr;
        }        
      }

      //  Deallocation function, will check if the the type has greater
      // alignement requirement than alignof(void*), if true, we read the
      // offset value from the byte located before the pointer argument
      // then we calculate the pointer difference to get the original
      // pointer returned by the allocation function, then we deallocate
      // using that pointer value.
      template<typename _Tp>
      void
      _M_deallocate(pointer _ptr, size_type _n_elem)
        noexcept(noexcept(_alloc_traits::deallocate({}, {}, {})))
      {
        if(_ptr == pointer{})
          return;

        static_assert( 0 < alignof(_Tp) && alignof(_Tp) <= 256
                    , "Unsupported Alignement.");
        constexpr std::size_t _M_align_val = alignof(_byte_type);
        
        if constexpr(alignof(_Tp) <= _M_align_val)
          return _alloc_traits::deallocate(_M_get_allocator(), _ptr, _n);
        else
        {
          const size_type _Align_diff = alignof(_Tp) - _M_align_val;
          const size_type _n_bytes = _n_elem * sizeof(_Tp) + _Align_diff;
          // read the stored value
          const ptrdiff_t _M_offset = static_cast<ptrdiff_t>(*(_ptr - 1));
          pointer _M_ptr = _ptr - _M_offset;
          _alloc_traits::deallocate(_M_get_allocator(), _M_ptr, _n_bytes);
          return;
        }
      }
    };

    // Maximum memory Arena size that could be allocated.
    constexpr std::size_t __Arena_sz = 4096UL;

    /**
     * _Coupled_vectors_base
     * base class for the main class 'Coupled_vectors'.
     * all main operations: grow, shrink, squeeze_to_fit, range_copy...etc
     * are implemented here.
     * A page of memory size (4096 bytes) is picked as a threshold for a 
     * contiguous arena allocation strategy.
     * this is an arbitrary choice not based on any benchmarking at this date.
     * the idea is to distribute the arena size over the count of types and 
     * their sizes, and the length of each buffer. but since the only compile
     * time information we can get are the count of types and their sizes, 
     * then we will use them to decide what allocation strategy we use:
     *       (count of types) * (sum of types sizes) <= Arena size
     *                   /                           \
     *               No /                             \ Yes
     *  Individual buffer allocation.            Arena based allocation 
     * 
     * While in Arena allocation strategy; if the increase in the number of
     * elements produce a growth of memory allocation larger thant what the
     * platform can provide; it is advisable to destory
    */
    template<typename _Alloc, typename... _Typs>
    struct _Coupled_vectors_base
    :public _Alloc_base<_Alloc
                      ,(sizeof...(_Typs) * (sizeof(_Typs)+...)) <= __Arena_sz>;
    
  } // namespace __detail
}// namespace stl
#endif // _COUPLED_VECTORS_H