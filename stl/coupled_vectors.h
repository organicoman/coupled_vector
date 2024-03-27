/* Vector implementation -*- C++ -*-

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
// <http://www.gnu.org/licenses/>.*/

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
#include <stdexcept>
#include <type_traits>
#include <memory>  // std::allocator_traits, std::pointer_traits
#include <cstring> // std::memcpy
#include <cstddef> // std::byte
#include <tuple>
#include <vector>
#include <algorithm> // std::transform
#include <cassert>

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
   *    couplvecs : or coupled vectors.
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
  class couplvecs : public std::vector<_Tp, _Alloc>
  { };

  namespace __detail
  {
    // helper type aliases
    template<typename _Alloc>
    struct __Value_type
    {
      template<typename _Head>
      struct _1st_arg_type
      { using __type = _Head; };

      template<typename ..._Ts>
      struct _1st_arg_type<std::tuple<_Ts...>>
      { using __type = typename std::tuple_element<0, std::tuple<_Ts...>>::type ;};

      using __type = typename _1st_arg_type<typename _Alloc::value_type>::__type;
    };

    template<typename _Alloc>
    using _get_value_type = typename __Value_type<_Alloc>::__type;

    // helper struct to customize alignement while keeping the size equal to 1
    // using an Alignement value not supported by the implementation is UB.
    template<std::size_t _N>
    struct aligned
    {
      using byte = struct alignas(_N) _{};
    };

    template<std::size_t _AlignOf>
    using _aligned_byte = typename aligned<_AlignOf>::byte;

    // helper alias type to create a type erased allocator.
    // The idea is to have an allocator that allocates One byte at different
    // alignement.
    template<typename _Alloc, typename _Byte_type>
    using _type_erased_alloc =
      typename std::allocator_traits<_Alloc>::template rebind_alloc<_Byte_type>;

    // type traits to check if an allocator is rebindable to its corresponding
    // type erased.
    template<typename _U, typename _Alloc>
    using _alloc_traits_rebind
      = typename std::allocator_traits<_Alloc>::template rebind_alloc<_U>;

    template<typename _A, typename _B>
    inline constexpr bool __is_rebindable_v = std::conjunction_v<
        std::is_same<_alloc_traits_rebind<typename _B::value_type, _A> ,_B>
      , std::is_same<_alloc_traits_rebind<typename _A::value_type, _B> ,_A>
        >;

    // type traits to detect if a type is a tuple (pair is also a tuple)
    template<typename _Tp>
    struct __is_tuple : std::false_type{};
    template<typename...Ts>
    struct __is_tuple<std::tuple<Ts...>> : std::true_type{};
    template<typename T, typename V>
    struct __is_tuple<std::pair<T,V>>: std::true_type{};

    template<typename T>
    inline constexpr bool __is_tuple_v = __is_tuple<T>::value;

    /*
     *
     * @brief _Alloc_base: base class to manage allocation of the coupled
     *        buffers.
     *        Two policies could be used; either allocate a contiguous arena
     *        to hold all buffers, Or allocate the necessary size for each
     *        buffer separatly. The minimum size to switch from arena to
     *        individual buffers, is user provided in derived class.
     *        The Arena policy is good for locality and cache hits. But
     *        could raise std::bad_array_new_length exception if the system
     *        cannot provide the requested size.
     * @tparam _Alloc: allocator type.
     *         We rebind this allocator to empty struct type, aligned at
     *         'alignof(void*)', memory boundaries.
     *         This is called type erased allocator. We use it to allocate the
     *         requested length of bytes. There are two policies of allocation:
     *           - Arena policy : we allocate a contiguous array of memory,
     *         then we calculate the offset of each coupled-buffer from the
     *         start of the array (type alignment and padding must
     *         be respected between each buffer's partition when calculating
     *         each partition size and begin pointer).
     *           - Spread policy : We allocate each buffer's memory array
     *         separately, respecting alignment.
     *
     *         The two policies cover two use cases. When locality of all
     *         buffers is required, use Arena policy; If the system cannot
     *         provide the requested length of bytes as a contiguous array,
     *         then switch to Spead policy, where each buffer contains unique
     *         type of elements.
    */

    template<typename _Alloc>
    struct _Alloc_base
    : public _type_erased_alloc<_Alloc, _aligned_byte<alignof(void*)>>
    {
      enum __storage_policy { _Spread  = false, _Arena = true};

      using _byte_type    = _aligned_byte<alignof(void*)>;
      using _base_type    = _type_erased_alloc<_Alloc, _byte_type>;
      using _alloc_traits = std::allocator_traits<_base_type>;
      using _value_type   = typename _alloc_traits::value_type;
      using _ptr_type     = typename _alloc_traits::pointer;
      using _size_type    = typename _alloc_traits::size_type;
      using _diff_type    = typename _alloc_traits::difference_type;

      static_assert(__is_rebindable_v<_Alloc, _base_type>,
        "Unable to type erase the given Allocator. Allocator must be "
        "rebindable to empty struct.");

      //class implementation

      /*
        * since _base_type and _Alloc are rebind related then
        * _base_type(_Alloc()) == _Alloc(_base_type())
        * see https://en.cppreference.com/w/cpp/named_req/Allocator
        *     Relationship between instances
      */

      constexpr
      _Alloc_base(const _Alloc& __other)
        noexcept(std::is_nothrow_copy_constructible_v<_base_type>
              && std::is_nothrow_constructible_v<_base_type, _Alloc>)
      : _base_type(
          std::allocator_traits<_Alloc>
          ::select_on_container_copy_construction(__other)
                  )
      {
        static_assert(__is_rebindable_v<_Alloc, _base_type>,
          "_Alloc<T> must be rebindable to _type_erased_alloc<std::byte>.");
      }

      constexpr
      _Alloc_base(_Alloc&& __other)
        noexcept(std::is_nothrow_move_constructible_v<_base_type>)
      : _base_type(std::move(__other))
      { }

      // rule of 5
      constexpr _Alloc_base() = default;

      constexpr
      _Alloc_base(const _Alloc_base& __other) = default;

      constexpr
      _Alloc_base(_Alloc_base&& __other) = default;

      constexpr
      _Alloc_base&
      operator= (const _Alloc_base& __other) = default;

      constexpr
      _Alloc_base&
      operator= (_Alloc_base&& __other) = default;

      _Alloc_base&
      _M_get_allocator() noexcept
      { return *this; }

      const _Alloc_base&
      _M_get_allocator() const noexcept
      { return *this; }

      template<typename _Tuple>
      using _Tuple_head = typename std::tuple_element<0, _Tuple>::type;

      template<typename _Tp, __storage_policy _Policy>
      [[nodiscard]]
      constexpr
      _ptr_type
      _M_allocate(_size_type _n_elem)
        noexcept(noexcept(_base_type{}.allocate({})))
      {
        auto _Self = _M_get_allocator();
        if constexpr(_Policy == __storage_policy::_Arena)
        {
          using _Tuple = _Tp;
          static_assert(__is_tuple_v<_Tuple>
              , "_M_allocate() for Arena policy takes a tuple-like type.");
          return _Allocate_hlpr<_Tuple>{}(_Self, _n_elem);
        }
        if constexpr(_Policy == __storage_policy::_Spread)
        {
          static_assert(not __is_tuple_v<_Tp>
          ,"_M_allocate() for Spread policy takes a non tuple-like type.");
          return _Allocate_hlpr<_Tp>{}(_Self, _n_elem);
        }
        // should not be reached.
        assert(false && "There are only two Storage policies!");
      }

      template<typename _Tp, __storage_policy _Policy>
      constexpr
      void
      _M_deallocate(_ptr_type& _ptr, _size_type _n_elem)
        noexcept(true)
      {
        auto _Self = _M_get_allocator();

        if constexpr(_Policy == __storage_policy::_Arena)
        {
          using _Tuple = _Tp;
          static_assert(__is_tuple_v<_Tuple>
              , "_M_deallocate() for Arena policy takes a tuple-like type.");
          return _Deallocate_hlpr<_Tuple>{}(_Self, _n_elem);
        }
        if constexpr(_Policy == __storage_policy::_Spread)
        {
          static_assert(not __is_tuple_v<_Tp>
          ,"_M_deallocate() for Spread policy takes a non tuple-like type.");
          return _Deallocate_hlpr<_Tp>{}(_Self, _n_elem);
        }
        // should not be reached.
        assert(false && "There are only two Storage policies!");
      }

      // helper functions
      template<typename _Last_type>
      static
      constexpr _size_type
      _M_byte_size(_size_type __last_offset, _size_type __n_elem)
      {
        return __last_offset + (__n_elem * sizeof(_Last_type));
      }

      template<typename..._Ts>
      static
      constexpr auto
      _M_nbytes_and_offsets(_size_type _n_elem) ->
            std::tuple<_size_type, std::array<_size_type, sizeof...(_Ts)>>
      {
        constexpr std::size_t _N = sizeof...(_Ts);
        static_assert(_N != 0, "At least one type must be provided!");

        constexpr std::array<_size_type, _N> _Szof = {sizeof(_Ts)...};
        constexpr std::array<_size_type, _N> _Algnof = {alignof(_Ts)...};

        // calculate total size in bytes
        std::array<_size_type, _N> _M_diffs{0};

        for(_size_type it = 1; it < _N; ++it)
        {
          auto v = _Szof[it-1] * _n_elem;
          auto cum = v + _M_diffs[it-1];
          auto mod = _Algnof[it] < alignof(_byte_type)
                    ? alignof(_byte_type) : _Algnof[it];
          if(cum % mod == 0)
            _M_diffs[it] = cum;
          else
            _M_diffs[it] = cum + (mod - (cum % mod));
        }
        // last type in the variadic args list
        using _Last_type =
          typename std::tuple_element<_N - 1, std::tuple<_Ts...>>::type;

        constexpr _size_type _n_bytes =
          _M_byte_size<_Last_type>(_M_diffs.back(), _n_elem);
        return {_n_bytes, _M_diffs};
      }

      /**
       * helper classes implementation
      */
      template<typename _Tp>
      struct _Allocate_hlpr
      {
        [[nodiscard]]
        constexpr
        _ptr_type
        operator()(_Alloc_base& _self, _size_type _n_elem) const
        {
          if(_n_elem == 0)
            return _ptr_type{};

          constexpr std::size_t _n_bytes = _n_elem * sizeof(_Tp);
          constexpr _diff_type _Align_diff = alignof(_Tp) - alignof(_byte_type);
          if constexpr(_Align_diff > 0)
          {
            /*
            * For alignment not covered by default implementation we need to be
            * able to retrieve the original pointer before alignement correction
            * thus we store an offset factor to the original pointer.
            */
            // adjusted number of byte for correct alignement
            constexpr std::size_t _n_bytes_adj = _n_bytes + _Align_diff;
            _ptr_type _ptr = _alloc_traits::allocate(_self, _n_bytes_adj);
            if(_ptr == _ptr_type{})
              return _ptr;
            // realign original pointer
            const _ptr_type _origin_ptr = _ptr;
            if(not std::align(alignof(_Tp), sizeof(_Tp), _ptr, _n_bytes_adj))
            {
              _alloc_traits::deallocate(_self, _ptr, _n_bytes_adj);
              // keep the original allocator exception behavior
              if constexpr(noexcept(_self.allocate({})))
                return _ptr_type{}; //nullable ptr
              else
                throw std::bad_alloc{};
            }
            /*
            * since the general rule for alignement is to use power of 2 values
            * we can prove that there is one integer x > 3 where 2^x = 8 * n
            * a multiple of 8. That is; any power of 2 alignement >= 8 could be
            * expressed by an alignement of 8.
            * we will store only the factor into one char variable.
            */
            _diff_type _factor = (_ptr - _origin_ptr) / alignof(_byte_type);
            if(_factor != 0 )
              _alloc_traits::construct(_self, reinterpret_cast<char*>(_ptr - 1)
                            , static_cast<char>(_factor));
            else
            { }// nothing to do, the pointer is already aligned
            return _ptr;
          }
          else
            // covered by default alignement
            return _alloc_traits::allocate(_self, _n_bytes);
        }
      };

      template<typename _Tp>
      struct _Deallocate_hlpr
      {
        constexpr
        void
        operator()(_Alloc_base& _self, _ptr_type _ptr, _size_type _n_elem)
          const noexcept // as per cpp standard
        {
          if(_ptr == _ptr_type{})
            return;
          /**
           * if the type's alignement is not covered by the allocator then
           * check if the pointer, to be deallocated, is correctly aligned for
           * _Tp alignement, if not then fetch the offset factor
          */
          constexpr _size_type _Align_diff = alignof(_Tp) - alignof(_byte_type);
          constexpr std::size_t _n_bytes = _n_elem * sizeof(_Tp);
          constexpr _size_type _n_bytes_adj = _n_bytes + _Align_diff;
          if constexpr(_Align_diff > 0)
          {
            if(_ptr % alignof(_Tp) == 0) // pointer correctly aligned
              return _alloc_traits::deallocate(_self, _ptr, _n_bytes_adj);

            const auto _factor = static_cast<char>(*(_ptr - 1));
            _ptr_type _origin_ptr = _ptr - (_factor * alignof(_byte_type));
            _alloc_traits::deallocate(_self, _origin_ptr, _n_bytes_adj);
          }
          else
            _alloc_traits::deallocate(_self, _ptr, _n_bytes);
        }
      };

      template<typename... _Ts>
      struct _Allocate_hlpr<std::tuple<_Ts...>>
      {
        // implementation
        [[nodiscard]]
        constexpr
        _ptr_type
        operator()(_Alloc_base& _self, _size_type _n_elem) const
        {
          if(_n_elem == 0)
            return _ptr_type{};

          using _1st_type = _Tuple_head<std::tuple<_Ts...>>;
          auto [_n_bytes, _diffs] = _M_nbytes_and_offsets<_Ts...>(_n_elem);
          constexpr _diff_type _Align_diff
              = alignof(_1st_type) - alignof(_byte_type);
          if constexpr(_Align_diff > 0)
          {
            /*
            * if the head-type of the tuple, has an alignment not covered by the
            * default implementation, we adjust alignment and store the offset.
            * the total size of the arena, is calculated in such a manner that
            * all buffers are aligned at their natural alignement, by adding
            * padding between buffers if necessary.
            */
            constexpr std::size_t _n_bytes_adj = _n_bytes + _Align_diff;
            _ptr_type _ptr = _alloc_traits::allocate(_self, _n_bytes_adj);
            if(_ptr == _ptr_type{})
              return _ptr;
            // realign original pointer
            const _ptr_type _origin_ptr = _ptr;
            if( not std::align(alignof(_1st_type)
                            , sizeof(_1st_type)
                            , _ptr, _n_bytes_adj) )
            {
              _alloc_traits::deallocate(_self, _ptr, _n_bytes_adj);
              // keep the original allocator exception behavior
              if constexpr(noexcept(_self.allocate({})))
                return _ptr_type{}; //nullable ptr
              else
                throw std::bad_alloc{};
            }
            /*
            * since the general rule for alignement is to use power of 2 values
            * we can prove that there is one integer x > 3 where 2^x = 8 * n
            * a multiple of 8. That is; any power of 2 alignement >= 8 could be
            * expressed by an alignement of 8.
            * we will store only the factor in a char variable, in memory one
            * byte before the actual pointer.
            */
            const _diff_type _factor = (_ptr - _origin_ptr) / alignof(_byte_type);
            if(_factor != 0 )
              _alloc_traits::construct(_self, reinterpret_cast<char*>(_ptr - 1)
                            , static_cast<char>(_factor));
            else
            { }// nothing to do, the pointer is already aligned
            return _ptr;
          }
          else
            // covered by default alignement
            return _alloc_traits::allocate(_self, _n_bytes);
        }
      };

      template<typename..._Ts>
      struct _Deallocate_hlpr<std::tuple<_Ts...>>
      {
        constexpr
        void
        operator()(_Alloc_base& _self, _ptr_type _ptr, _size_type _n_elem)
        const noexcept
        {
          if(_ptr == _ptr_type{})
            return;
          using _1st_type = _Tuple_head<std::tuple<_Ts...>>;
          auto [_n_bytes, _M_diffs] = _M_nbytes_and_offsets<_Ts...>(_n_elem);
          constexpr _diff_type _Align_diff
              = alignof(_1st_type) - alignof(_byte_type);
          if constexpr(_Align_diff > 0)
          {
            constexpr std::size_t _n_bytes_adj = _n_bytes + _Align_diff;
            if(_ptr % alignof(_1st_type) == 0) // pointer correctly aligned
              return _alloc_traits::deallocate(_self, _ptr, _n_bytes_adj);

            const auto _factor = static_cast<char>(*(_ptr - 1));
            _ptr_type _origin_ptr = _ptr - (_factor * alignof(_byte_type));
            _alloc_traits::deallocate(_self, _origin_ptr, _n_bytes_adj);
          }
          else
            _alloc_traits::deallocate(_self, _ptr, _n_bytes);
        }
      };
    };

    /**
     * coupled_vectors base Implementation
    */
    template<typename _Alloc, typename... _Ts>
    struct _couplvecs_Impl : _Alloc_base<_Alloc>
    {
      // typedefs
      using _allocator_base = _Alloc_base<_Alloc>;
      using _pointer = typename _allocator_base::_ptr_type;
      using _size_t  = typename _allocator_base::_size_type;
      using _data_ptrs_array = std::array<_pointer, sizeof...(_Ts)>;

      // data members
      _data_ptrs_array _M_arr;
      _size_t          _M_capacity;

      // rule of 5
      constexpr
      _couplvecs_Impl() = default;

      constexpr
      _couplvecs_Impl(const _couplvecs_Impl& _other) = default;

      constexpr
      _couplvecs_Impl(_couplvecs_Impl&& _other) = default;

      constexpr
      _couplvecs_Impl&
      operator=(const _couplvecs_Impl& _other) = default;

      constexpr
      _couplvecs_Impl&
      operator=(_couplvecs_Impl&& _other) = default;

      // conversion from _Alloc
      constexpr
      _couplvecs_Impl(const _Alloc& __alloc)
      : _allocator_base(
          std::allocator_traits<_Alloc>
                    ::select_on_container_copy_construction(__alloc)
                       )
      , _M_arr{}
      , _M_capacity(0)
      { }

      // conversion from _Alloc
      constexpr
      _couplvecs_Impl(_Alloc&& __alloc)
      : _allocator_base(std::move(__alloc))
      , _M_arr{}
      , _M_capacity(0)
      { }

      // the destructor is implicit and trivial, and the deallocation
      // of storage is done in the derived class, since it relys on the
      // type of storage policy (Arena, Spread) which is not known in this class.

      constexpr
      _Alloc&
      __get_alloc() noexcept
      { return _Alloc(this->_M_get_allocator()); }

      constexpr
      const _Alloc&
      __get_alloc() const noexcept
      { return _Alloc(this->_M_get_allocator()); }

      constexpr
      void
      _M_init_buffers(const _data_ptrs_array& __ptrs)
        noexcept
      { this->_M_arr = __ptrs; }

      // given the pointer to the first byte of the Arena memory storage,
      // calculate and store the pointers offsets for each buffer in the arena.
      constexpr
      void
      _M_init_buffers(_pointer __head
                    , const std::array<_size_t, sizeof...(_Ts)>& __offsets)
        noexcept
      {
        std::transform(__offsets.cbegin() , __offsets.cend()
                     , this->_M_arr.begin()
                     , [__head](_size_t _off)
                     { return __head + _off; });
      }

      /***
       * @brief
       *    create storage to hold exactly n_elem for each buffer type.
       *    the pointer to the first byte of each buffer is stored in
       *    the std::array data member. The capacity is updated to _n_elem value.
       * @tparam _n_elem:
       *    number of elements common to all types buffers.
       * @tparam __Arena_policy:
       *    true  : uses arena storage allocation policy
       *    false : uses sprea storage allocation policy
       * @return
       *    true if the allocation succeeds, false otherwise.
       */
      [[maybe_unused]]
      bool
      _M_create_storage(_size_t _n_elem, bool __Arena_policy) const
      {
        _data_ptrs_array __tmp {_pointer{}};

        if(__Arena_policy)
        {
          _pointer __beg
              = this->template _M_allocate<std::tuple<_Ts...>, true>(_n_elem);

          if(__beg == _pointer{})
            return false;

          const std::array<_size_t, sizeof...(_Ts)>
            __offsets = this->_M_nbytes_and_offsets(_n_elem);

          std::transform(__offsets.cbegin() , __offsets.cend(), __tmp.begin()
                        , [__beg](_size_t _off){ return __beg + _off; });
        }
        else
        {
          try
          {
            // pack expansion into an initialization list.
            __tmp = { this->template _M_allocate<_Ts, false>(_n_elem)... };

            // if any of the coupled buffers failed to allocate. then
            // deallocate all and propagate exception.
            for(auto ptr: __tmp)
              if(ptr == _pointer{})
                throw;
          }
          catch(...)
          {
            // pack expansion deallocation.
            std::size_t i = 0;
            (this->template _M_deallocate<_Ts, false>(__tmp[i++], _n_elem), ...);
            // propagate exception
            throw std::bad_alloc();
          }
        }
        this->_M_capacity = _n_elem;
        this->_M_arr = std::move(__tmp);
        return true;
      }

      /***
       *
       */

      [[nodiscard]]
      std::pair<_size_t, _data_ptrs_array>
      _M_grow_storage(_size_t _prev_cap, bool __Arena_policy) const
      try
      {
        // capcity: find the closest power of 2 > __prev_cap_doubled
        _size_t _cap = 1;
        const auto __prev_cap_doubled = 2 * _prev_cap;
        while(_cap <= __prev_cap_doubled) _cap <<= 1;
        return this->_M_create_storage(_cap, __Arena_policy);
      }
      catch(...)
      { throw; }

      // if the new size is Zero, deallocate all buffers
      [[nodiscard]]
      std::pair<_size_t, _data_ptrs_array>
      _M_shrink_storage(_size_t _prev_cap, _size_t __n, bool __Arena_policy)
      try
      {
        if(__n != 0)
          return this->_M_create_storage(__n, __Arena_policy);

        // User request deallocation of the current storage.
        // User has the responsabilty to destroy any element befor calling this
        // member function.
        if(__Arena_policy)
          this->template _M_deallocate<std::tuple<_Ts...>, true> (this->_M_arr[0], _prev_cap);
        else
        {
          std::size_t i = 0;
          (this->template _M_deallocate<_Ts, false> (this->_M_arr[i++], _prev_cap), ...);
          this->_M_arr = {_pointer()};
        }
        return {0, {_pointer()}};
      }
      catch(...)
      { throw; }

      template<typename _Tp>
      struct _iterator;

      template<typename _Tp>
      struct _const_iterator;

    }; //_couplvecs_Impl
  }// namespace __detail


  // partial specialization
  // std::pair is a special case of std::tuple
  template<typename..._Ts, typename _Alloc>
  class couplvecs<std::tuple<_Ts...>, _Alloc>
    : private __detail::_couplvecs_Impl<_Alloc, _Ts...>
  {
    /**
     * typedefs
    */
   private:
    using __base_type  = __detail::_couplvecs_Impl<_Alloc, _Ts...>;
    using __base_alloc = typename __base_type::_allocator_base;

    template<typename _T>
    using __iterator   = typename __base_type::template _iterator<_T>;

    template<typename _T>
    using __const_iter = typename __base_type::template _const_iterator<_T> ;

   public:
    using allocator_type        = __base_alloc;
    using size_type             = std::size_t;
    using difference_type       = std::ptrdiff_t;
    using value_type            = std::tuple<_Ts...>;
    using reference             = std::tuple<_Ts&...>;
    using const_reference       = std::tuple<const _Ts&...>;
    using pointer               = std::tuple<_Ts*...>;
    using const_pointer         = std::tuple<const _Ts*...>;
    using iterator              = std::tuple<__iterator<_Ts>...>;
    using const_iterator        = std::tuple<__const_iter<_Ts>...>;

    template<std::size_t _Idx>
    using channel_val_type      = std::tuple_element_t<_Idx, value_type>;
    template<std::size_t _Idx>
    using channel_ref           = std::tuple_element_t<_Idx, reference>;
    template<std::size_t _Idx>
    using channel_const_ref     = std::tuple_element_t<_Idx, const_reference>;
    template<std::size_t _Idx>
    using channel_pointer       = std::tuple_element_t<_Idx, pointer>;
    template<std::size_t _Idx>
    using const_channel_pointer = std::tuple_element_t<_Idx, const_pointer>;
    template<std::size_t _Idx>
    using channel_iterator      = std::tuple_element_t<_Idx, iterator>;
    template<std::size_t _Idx>
    using channel_const_iterator= std::tuple_element_t<_Idx, const_iterator>;

// data members.
  private:
    std::size_t _m_max_arena;
    size_type   _m_size;

  #ifdef NDEBUG
    enum _Err_Code
    {
      ALLOC_EXCEPT,
      CTOR_EXCEPT,
      DTOR_EXCEPT,
      MOVE_EXCEPT,
      COPY_EXCEPT,
      ASSIGN_EXCEPT,
      INDEX_EXCEPT
    }           _m_last_error;

    void
    _M_set_last_error(_Err_Code __err) noexcept
    { _m_last_error = __err; }

    _Err_Code
    _M_last_error() const noexcept
    { return _m_last_error; }

  #endif


   public:
    // Member functions

    /**
     * @tparam Arena_max_bytes :
     *         Threshold in bytes of memory, after which the implementation
     *         switchs from Arena storage policy to Spread storage policy.
     *         As a default value, we use the size of allocating 20 elements of
     *         each type. Usually 20 elements can be linearly traversed in
     *         effecient time.
    */
    template<std::size_t Arena_max_bytes = 20 * (sizeof(_Ts)+...)>
    constexpr
    couplvecs() noexcept(noexcept(_Alloc()))
    : __base_type(), _m_max_arena(Arena_max_bytes), _m_size(0)
    { }

    template<std::size_t Arena_max_bytes = 20 * (sizeof(_Ts)+...)>
    constexpr
    explicit
    couplvecs(const _Alloc& alloc) noexcept
    : __base_type(alloc), _m_max_arena(Arena_max_bytes), _m_size(0)
    { }

    template<std::size_t Arena_max_bytes = 20 * (sizeof(_Ts)+...)>
    constexpr
    explicit
    couplvecs(size_type count, const_reference value
            , const _Alloc& alloc = _Alloc())
    : __base_type(count, value, alloc), _m_max_arena(Arena_max_bytes), _m_size(0)
    { }

    template<std::size_t Arena_max_bytes = 20 * (sizeof(_Ts)+...)>
    constexpr
    explicit
    couplvecs(size_type count, const _Alloc& alloc = _Alloc())
    : __base_type(count, alloc), _m_max_arena(Arena_max_bytes), _m_size(0)
    { }

    template<typename InputIt
             , std::size_t Arena_max_bytes = 20 * (sizeof(_Ts)+...)>
    constexpr
    couplvecs(InputIt first, InputIt last, const _Alloc& alloc = _Alloc())
    : __base_type(first, last, alloc), _m_max_arena(Arena_max_bytes), _m_size(0)
    { }

    template<std::size_t Arena_max_bytes = 20 * (sizeof(_Ts)+...)>
    constexpr
    couplvecs(const couplvecs& other)
    : __base_alloc(other), _m_max_arena(Arena_max_bytes), _m_size(0)
    { }

    template<std::size_t Arena_max_bytes = 20 * (sizeof(_Ts)+...)>
    constexpr
    couplvecs(const couplvecs& other, const _Alloc& alloc)
    : __base_type(other, alloc), _m_max_arena(Arena_max_bytes), _m_size(0)
    { }

    template<std::size_t Arena_max_bytes = 20 * (sizeof(_Ts)+...)>
    constexpr
    couplvecs(couplvecs&& other) noexcept
    : __base_alloc(std::move(other)), _m_max_arena(Arena_max_bytes), _m_size(0)
    { }

    template<std::size_t Arena_max_bytes = 20 * (sizeof(_Ts)+...)>
    constexpr
    couplvecs(couplvecs&& other, const _Alloc& alloc)
    : __base_type(std::move(other), alloc)
    { }

    template<std::size_t Arena_max_bytes = 20 * (sizeof(_Ts)+...)>
    constexpr
    couplvecs(std::initializer_list<value_type> init
            , const _Alloc& alloc = _Alloc())
    : __base_type(std::move(init), alloc)
    { }

    // Element access
    constexpr reference
    at(size_type _Idx)
    {
      if(_Idx >= size())
      {
        #ifdef NDEBUG
          _M_set_last_error(_Error_Code::INDEX_EXCEPT);
        #endif
        throw std::out_of_range("");
      }
      return (*this)[_Idx];
    };

    constexpr const_reference
    at(size_type _Idx) const
    {
      if(_Idx >= size())
      {
        #ifdef NDEBUG
          _M_set_last_error(_Error_Code::INDEX_EXCEPT);
        #endif
        throw std::out_of_range("");
      }
      return (*this)[_Idx];
    }

    constexpr reference
    operator[](size_type _Idx)
    {
      #ifdef NDEBUG
        _M_set_last_error(_Error_Code::INDEX_EXCEPT);
      #endif
      std::size_t __i = 0;
      return
        { ( *((reinterpret_cast<_Ts*>(this->_M_arr[__i++])) + _Idx) )... };
    }

    constexpr const_reference
    operator[](size_type _Idx) const
    {
      #ifdef NDEBUG
        _M_set_last_error(_Error_Code::INDEX_EXCEPT);
      #endif
      std::size_t __i = 0;
      return
        {(* ((reinterpret_cast<const _Ts*>(this->_M_arr[__i++])) + _Idx) )... };
    }

    [[deprecated("NotImplemented")]]
    constexpr reference
    front() noexcept;

    [[deprecated("NotImplemented")]]
    constexpr const_reference
    front() const noexcept;

    [[deprecated("NotImplemented")]]
    constexpr reference
    back() noexcept;

    [[deprecated("NotImplemented")]]
    constexpr const_reference
    back() const noexcept;

    [[deprecated("NotImplemented")]]
    constexpr pointer
    data() noexcept;

    [[deprecated("NotImplemented")]]
    constexpr const_pointer
    data() const noexcept;

    // Iterators
    [[deprecated("NotImplemented")]]
    constexpr iterator
    begin() noexcept
    { }

    [[deprecated("NotImplemented")]]
    constexpr const_iterator
    begin() const noexcept;

    [[deprecated("NotImplemented")]]
    constexpr const_iterator
    cbegin() const noexcept;

    [[deprecated("NotImplemented")]]
    constexpr iterator
    end() noexcept;

    [[deprecated("NotImplemented")]]
    constexpr const_iterator
    end() const noexcept;

    [[deprecated("NotImplemented")]]
    constexpr const_iterator
    cend() const noexcept;

    [[deprecated("NotImplemented")]]
    constexpr reverse_iterator
    rbegin() noexcept;

    [[deprecated("NotImplemented")]]
    constexpr const_reverse_iterator
    rbegin() const noexcept;

    [[deprecated("NotImplemented")]]
    constexpr const_reverse_iterator
    crbegin() const noexcept;

    [[deprecated("NotImplemented")]]
    constexpr reverse_iterator
    rend() noexcept;

    [[deprecated("NotImplemented")]]
    constexpr const_reverse_iterator
    rend() const noexcept;

    [[deprecated("NotImplemented")]]
    constexpr const_reverse_iterator
    crend() const noexcept;

    // true when all buffers are empty.
    constexpr bool
    empty() const noexcept
    { return _m_size == 0; }

    // number of elements common to all buffers
    constexpr size_type
    size() const noexcept
    { return _m_size; }

    // number of elements that could be reached before reallocation
    constexpr size_type
    capacity() const noexcept
    { return this->_M_capacity; }

    // number of buffers
    constexpr std::size_t
    count() const noexcept
    { return sizeof...(_Ts); }

    // size in bytes of the memory occupied by all buffers
    constexpr size_type
    length() const noexcept
    { return capacity() * ((sizeof(_Ts)+...)) ; }

    [[deprecated("NotImplemented")]]
    constexpr size_type
    max_size() const noexcept;

    [[deprecated("NotImplemented")]]
    constexpr void
    reserve(size_type _n_elem);

    [[deprecated("NotImplemented")]]
    constexpr bool
    squeeze() noexcept;

    // Modifiers
    [[deprecated("NotImplemented")]]
    constexpr bool
    clear() noexcept;

    [[deprecated("NotImplemented")]]
    constexpr bool
    insert();

    [[deprecated("NotImplemented")]]
    constexpr bool
    insert_range();

    [[deprecated("NotImplemented")]]
    constexpr bool
    emplace();

    [[deprecated("NotImplemented")]]
    constexpr bool
    erase();

    [[deprecated("NotImplemented")]]
    constexpr bool
    push_back(const_reference _val);

    [[deprecated("NotImplemented")]]
    constexpr bool
    push_back(value_type&& _rval);

    [[deprecated("NotImplemented")]]
    constexpr bool
    append_range();

    [[deprecated("NotImplemented")]]
    constexpr bool
    pop_back();

    [[deprecated("NotImplemented")]]
    constexpr bool
    resize();

    [[deprecated("NotImplemented")]]
    constexpr void
    swap();

    // Non-member functions
    [[deprecated("NotImplemented")]]
    friend bool operator==(const couplvecs& lft, const couplvecs& rht)
    { }

    [[deprecated("NotImplemented")]]
    friend bool operator!=(const couplvecs& lft, const couplvecs& rht)
    { return not (lft == rht); }

    [[deprecated("NotImplemented")]]
    friend void swap(couplvecs& lft, couplvecs rht)
    { }
  };
}// namespace stl
#endif // _COUPLED_VECTORS_H
