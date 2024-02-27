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

namespace stl
{
  /**
   * Vector of Structs or Struct of Vectors?
   * coupled_vectors : is a class template to manage many vector-like buffers
   * that are tied together.
   * All managed buffers grow and shrink together.
   * Adding elements works in packed form .i.e the user provide a value
   * to each underlying buffer.
   * Since this class manages many buffers, which are dissociated from each
   * others, yet coupled by operations (adding, erasing, growing..etc),
   * it makes it a good candidate for parallelism and vectorization techniques
   * 
   * The model is based and how GPU's work: 
   *    Many dissociated buffers vs One global threaded operation.
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
    /// @brief _Ptrs_array_base: A base class storing the start pointers
    ///        to several buffers.
    ///        The idea here, is to store pointers to buffers of different types,
    ///        and since the std::array accepts only one type as a typed template
    ///        parameter, then we will store all pointers as void pointers.
    ///        Later on when we need to access the elements of the buffers,
    ///        we static cast back to the original data type.
    /// @tparam N : an unsigned integer representing the number of coupled
    ///         vectors.
    template<std::size_t _Nm>
    struct _Ptrs_array_base :  std::array<void*, _Nm>
    {
      using  _void_ptr = void*;
      using _base_type = std::array<_void_ptr, _Nm>;

      using _base_type::_base_type;

      template<typename _Ptr_t>
      constexpr 
      explicit _Ptrs_array_base(std::array<_Ptr_t, _Nm> const& __arr)
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

    // helper alias type
    template<typename _Alloc, typename _Tp>
    using _rebind_Allocator = 
    typename std::allocator_traits<_Alloc>::template rebind<_Tp>::other;
    
    // helper struct for correct alignement
    template<std::size_t _Alignof>
    struct alignas(_AlignOf) aligned_byte
    {
      std::byte _block;
    };

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
    ///         use same implementation as std::vector.
    /// @tparam _Use_arena: bool value. True for allocating an Arena where
    ///         all buffers live together. False for individual allocations
    ///         for each buffer.
    
    // primary base class
    template<typename _Allocator, bool _Use_arena>
    struct _Alloc_base;

    // Arena allocation strategy
    template<typename _Alloc
           , std::size_t _Alignof = alignof(typename _Alloc::type_value)>
    struct _Alloc_base<_Alloc, true>
    : public _rebind_Allocator<_Alloc, aligned_byte<_AlignOf>> 
    {
      using _base_type = _rebind_Allocator<_Alloc, aligned_byte<_AlignOf>>;
      using _alloc_traits = std::allocator_traits<_base_type>;
      using value_type = typename _alloc_traits::value_type;
      using pointer = typename _alloc_traits::pointer;
      using size_type = typename _alloc_traits::size_type;

      // base type import all Ctors
      using _base_type::_base_type;

      template<typename..._Ts, size_type _N = sizeof...(_Ts)>
      auto _M_allocate(size_type _n_elem)
                      -> std::pair<pointer, std::array<pointer, _N>>
      {
        std::array<size_type, _N> _M_diffs{0};
        std::array<pointer, _N> _M_offsets{};
        
        // calculate total size in std::byte
        constexpr std::array<size_type, _N> _Szof = {sizeof(_Ts)...};
        constexpr std::array<size_type, _N> _Algnof = {alignof(_Ts)...};
        for(size_type it = 1; it < _N; ++it)
        {
          auto v = _Szof[it-1] * _n_elem;
          auto cum = v + _M_diffs[it-1];
          if(cum % _Algnof[it] == 0)
            _M_diffs[it] = cum;
          else
            _M_diffs[it] = cum + (_Algnof[it] - (cum % _Algnof[it]));
        }
        const size_type _n_bytes = _M_diffs.back() + _Szof.at(_N - 1) * _n_elem;
        const pointer _M_ptr = _alloc_traits::allocate(*this, _n_bytes);
        
        if(_M_ptr == pointer{})
          return {_M_ptr, _M_offsets};

        std::transform(_M_diffs.begin(), _M_diffs.end(), _M_offsets.begin()
                      , [_M_ptr](size_type offset){return _M_ptr + offset;});
        // for(size_type it = 0; it < _N; ++it)
        //   _M_offsets[it] = _M_ptr + _M_diffs[it];
        return {_M_ptr, _M_offsets};
      }
    };

    // partial specialization 
    // individual allocation strategy
    // using void* alignement as the default allocation alignement
    template<typename _Alloc>
    struct _Alloc_base<_Alloc, false> 
    : public _rebind_Allocator<_Alloc, aligned_byte<alignof(void*)>>
    {
      using _byte_type = aligned_byte<alignof(void*)>;
      using _base_type = _rebind_Allocator<_Alloc, _byte_type>;
      using _alloc_traits = std::allocator_traits<_base_type>;
      using pointer = typename _alloc_traits::pointer;
      using size_type = typename _alloc_traits::size_type;
      
      // Assumption:
      // alignof(_Tp) <= 255.
      template<typename _Tp>
      pointer _M_allocate(size_type _n_elem)
      {
        static_assert(alignof(_Tp) > 0xFF, "Unsupport Alignement.");

        constexpr std::size_t _M_align_val = alignof(_byte_type);
        if constexpr (alignof(_Tp) <= _M_align_val)
          return _alloc_traits::allocate(*this, _n_elem * sizeof(_Tp));
        // for any other specific alignement, we need to re-adjust the
        // request byte size, and keep track of the old pointer
        else 
        {
          const size_type _Align_diff = alignof(_Tp) - _M_align_val;
          size_type _n_bytes = _n_elem * sizeof(_Tp) + _Align_diff;
          const auto _old_n = _n_bytes;
          
          pointer _M_ptr = _alloc_traits::allocate(*this, _n_bytes);
          // failed allocation
          if(_M_ptr == pointer{})
            return pointer{};

          // adjust pointer according to alignement
          const pointer _M_old_ptr = _M_ptr;
          if(not std::align(alignof(_Tp), sizeof(_Tp)
                      , static_cast<void*&>(_M_ptr), _n_bytes))
          {
            _alloc_traits::deallocate(*this, _M_ptr, _n_bytes);
            // keep the behavior of the base allocator class
            // if it throws then throw.
            if constexpr (noexcept(this->allocate({})))
              return pointer();
            else
              throw std::bad_alloc();
          }
          // for successfull realignement, store offset from new pointer.
          // By guess work: a extended alignement of a value more than 255
          //    is not used, thus it is acceptable to reserve one byte
          //    to store the offset.
          const ptrdiff_t _M_offset = _M_ptr - _M_old_ptr;

          _alloc_traits::construct(*this, _M_ptr - 1
                                  , static_cast<std::byte>(_M_offset));
          return _M_ptr;
        }        
      }

      template<typename _Tp>
      void
      _M_deallocate(pointer _ptr, size_type _n_elem)
      noexcept(this->deallocate({}, {}))
      {
        static_assert(alignof(_Tp) > 0xFF, "Unsupported Alignement.");
        constexpr std::size_t _M_align_val = alignof(_byte_type);
        if constexpr(alignof(_Tp) <= _M_align_val)
          return _alloc_traits::deallocate(*this, _ptr, _n);
        else
        {
          const size_type _Align_diff = alignof(_Tp) - _M_align_val;
          const size_type _n_bytes = _n_elem * sizeof(_Tp) + _Align_diff;
          // read the stored value
          const ptrdiff_t _M_offset = static_cast<ptrdiff_t>(*(_ptr - 1));
          pointer _M_ptr = _ptr - _M_offset;
          _alloc_traits::deallocate(*this, _M_ptr, _n_bytes);
          return;
        }
      }

    };
    
  } // namespace __detail
}// namespace stl
#endif // _COUPLED_VECTORS_H