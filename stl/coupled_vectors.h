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
#include <tuple>

namespace stl
{
  template<typename _Alloc = std::allocator<std::byte>>
  struct _Coupled_vectors_base
  {
/// @brief _ptrs_store_base: A base class storing the start pointers
  ///        to several buffers.
  ///        The idea here, is to store pointers to buffers of different types,
  ///        and since the std::array accepts only one type as a typed template
  ///        parameter, then we will store all pointers as void pinters.
  ///        Later on when we need to access the elements of the buffers,
  ///        we static cast back to the original data type.
  /// @tparam N : an unsigned integer representing the number of coupled
  ///         vectors.

  template<std::size_t _Nm>
  struct _Ptrs_store_base
  {
    using _void_ptr = void*;
    using _ptrs_array_t = std::array<_void_ptr, _Nm>;

    _ptr_store_t _M_begins;
    std::size_t _M_sz; // elements count for all coupled vectors
    std::size_t _M_cap; // buffer capacity for all coupled vectors

    constexpr _Ptrs_store_base() noexcept
    : _M_begins(_ptrs_array_t{}) // List init, value initialize to nullptr
    , _M_sz(0)
    , _M_cap(0)
    { }

    ~_base_data() = default;
    
    constexpr 
    _Ptrs_store_base(const _Ptrs_store_base& __othr) noexcept = default;
    
    constexpr _Ptrs_store_base(_Ptrs_store_base&& __othr) noexcept
    : _M_begins(std::exchange(__othr._M_begins, _ptr_store_t{}))
    , _M_sz(std::exchange(__othr._M_sz, 0))
    , _M_sz(std::exchange(__othr._M_cap, 0))
    { }

    constexpr _Ptrs_store_base& 
    operator=(const _Ptrs_store_base& __rght) noexcept = default;

    constexpr _Ptrs_store_base& 
    operator=(_Ptrs_store_base&& __rght) noexcept
    {
      if(__rght == *this)
        return *this;
      _M_begins = std::exchange(__rght._M_begins, _ptr_store_t{});
      _M_sz = std::exchange(__rght._M_sz, 0);
      _M_cap = std::exchange(__rght._M_cap, 0);
      return *this;
    }

    constexpr 
    explicit _ptrs_store_base(std::size_t __sz, std::size_t __cap)
    : _M_begins(_ptrs_array_t{})
    , _M_sz(__sz)
    , _M_cap(__cap)
    { }

    template<typename _Ptr_t>
    constexpr 
    explicit _ptrs_store_base(std::array<_Ptr_t, _Nm> const& __begins
                            , std::size_t __sz, std::size_t __cap)
    : _M_sz(__sz)
    , _M_cap(__cap)
    {
      static_assert(std::is_pointer_v<_Ptr_t>
                   , "all args must be of pointer-type to data store.");
      std::memcpy(_M_begins.data(), __begins.data(), _Nm);
    }
    template<typename... _Ptr_t>
    constexpr _ptrs_store_base(_Ptr_t... __bufs)
    : _M_begins{static_cast<_void_ptr>(__bufs), ...}
    , _M_sz(0)
    , _M_cap(0)
    { 
      static_assert(sizeof...(_Ptr_t) != _Nm
                  , "number of buffers must be equal"
                    "to the predefined size of this struct");
      static_assert(std::is_pointer_v<_Ptr_t>&& ...
                  , "all args must be of pointer-type to data store.");
    }

    auto begin() noexcept
    { return std::begin(_M_begins); }

    auto end() noexcept
    { return std::end(_M_begins); }

    auto cbegin() const noexcept
    { return std::cbegin(_M_begins); }

    auto cend() const noexcept
    { return std::cend(_M_begins); }

    void _M_swap(_ptrs_store_base& __othr) noexcept
    {
      _M_begins = std::exchange(__othr._M_begins, _M_begins);
      _M_sz = std::exchange(__othr._M_sz, _M_sz);
      _M_cap = std::exchange(__othr._M_cap, _M_cap);
    }

    std::size_t 
    _M_coupled_vecs_count() const
    { return _Nm; }

    void 
    _M_clear() noexcept
    { 
      _M_begins = _ptr_store_t{0};
      _M_sz = 0;
      _M_cap = 0;
    }

    /**
     * the capacity of one vector buffer
    */
    std::size_t 
    _M_vec_cap() const noexcept
    { return _M_cap; }

    /**
     * the number of elements of one vector buffer
    */
    std::size_t 
    _M_vec_size() const noexcept
    { return _M_sz;}

    // not throwing, but possible UB
    // individual buffers are meant to change together, thus
    // this operator is not modifing.
    constexpr auto 
    operator[](std::size_t __idx) const noexcept
    { 
#ifdef __cplusplus >=201811L // compile time throw exception
      if (is_constant_evaluated())
        static_assert(__idx <= _Nm, "out of range constexpr index access.");
#endif
      return _M_begins[__idx]; 
    }
    
  }; // _Ptr_store_base
  

  template<typename _Alloc = std::allocator<std::byte>>
  struct _Alloc_base
  {

  }; // _Alloc_base

  }; // _Coupled_vectors_base

}// namespace stl
#endif // _COUPLED_VECTORS_H