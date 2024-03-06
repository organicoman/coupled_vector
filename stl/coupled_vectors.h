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
#include <variant>
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

    // helper type aliases
    template<typename _Alloc>
    struct __Value_type
    {
      template<typename _Head>
      struct _1st_arg_type
      { using __type = _Head; };
      template<typename ..._Ts>
      struct _1st_arg_type<std::tuple<_Ts...>>
      { using __type = typename std::tuple_element<0, std::tuple<_Ts...>::type ;};

      using __type = typename _1st_arg_type<_Alloc::value_type>::__type;
    };

    template<typename _Alloc>
    using _get_value_type = typename __Value_type<_Alloc>::__type;

    // helper struct to customize alignement while keeping the size equal to 1
    // using an Alignement value not supported by the implementation is UB.
    template<std::size_t _N>
    struct aligned
    {
      using byte = struct alignas(_N) type{};      
    };

    template<std::size_t _AlignOf>
    using _aligned_byte = typename aligned<_AlignOf>::byte;

    // helper alias type to rebind any allocator to its similar duplicate but
    // for 'byte' type.
    // The idea is to have an allocator that allocates One byte at different
    // alignement.
    template<typename _Alloc, typename _Byte_type>
    using _rebind_Allocator = 
      typename std::allocator_traits<_Alloc>::rebind_alloc<_Byte_type>; 

    // type traits to detect if a type is a tuple (pair is also a tuple)
    template<typename _Tp>
    struct __is_tuple : std::false_type{};
    template<typename...Ts>
    struct __is_tuple<std::tuple<Ts...>> : std::true_type{};
    template<typename T, typename V>
    struct __is_tuple<std::pair<T,V>>: std::true_type{};

    template<typename T>
    inline constexpr bool __is_tuple_v = __is_tuple<T>::value;   

    /// @brief _Alloc_base: base class to manage allocation of the coupled
    ///        buffers.
    ///        Two policies could be used; either allocate a contiguous arena
    ///        to hold all buffers, Or allocate the necessary size for each
    ///        buffer separatly. The minimum size to switch from arena to
    ///        individual buffers, is user provided in derived class.
    ///        The Arena policy is good for locality and cache hits. But
    ///        could raise std::bad_array_new_length exception if the system
    ///        cannot provide the requested size.
    /// @tparam _Alloc: allocator type. 
    ///         We rebind this allocator to 'unsinged char' type aligned at 
    ///         alignof(void*) memory boundaries.
    ///         For arena policy : we allocate a contiguous array of memory, 
    ///         then we calculate the offset of each coupled-buffer from the
    ///         start of the array (type alignment and padding must
    ///         be respected between each buffer's partition when calculating
    ///         each partition size and begin pointer).
    ///         For spread policy : We allocate each buffer's memory array 
    ///         separately, respecting alignment.
    
    template<typename _Alloc>
    struct _Alloc_base
    : public _rebind_Allocator<_Alloc, aligned_byte<alignof(void*)>> 
    {
      private:
      enum __memory_policy { _Spread  = false, _Arena = true};

      public:
      using _byte_type    = aligned_byte<alignof(void*)>;
      using _base_type    = _rebind_Allocator<_Alloc, _byte_type>
      using _alloc_traits = std::allocator_traits<_base_type>;
      using _value_type   = typename _alloc_traits::value_type;
      using _ptr_type     = typename _alloc_traits::pointer;
      using _size_type    = typename _alloc_traits::size_type;
      using _diff_type    = typename _alloc_traits::difference_type;

      /**
       * class implementation
      */

      // base type import all Ctors
      using _base_type::_base_type;

      _Alloc_base& 
      _M_get_allocator() noexcept
      { return *this; }

      const _Alloc_base& 
      _M_get_allocator() const noexcept
      { return *this; }

      template<typename _Tuple>
      using _Tuple_head = typename std::tuple_element<0, _Tuple>::type;

      template<typename _Tp, __memory_policy _Policy>
      [[nodiscard]]
      constexpr
      _ptr_type
      _M_allocate(_size_type _n_elem) 
        noexcept(noexcept(_base_type{}.allocate({})))
      try
      {
        auto _Self = _M_get_allocator();
        if constexpr(_Policy == __memory_policy::_Arena)
        {
          using _Tuple = _Tp;
          static_assert(__is_tuple_v<_Tuple>
              , "_M_allocate() for Arena policy takes a tuple-like type.");
          return _Allocate_hlpr<_Tuple>{}(_Self, _n_elem);
        }
        if constexpr(_Policy == __memory_policy::_Spread)
        {
          static_assert(not __is_tuple_v<_Tp>
          ,"_M_allocate() for Spread policy takes a non tuple-like type.");
          return _Allocate_hlpr<_Tp>{}(_Self, _n_elem);
        }
        // should not be reached.
        throw;
      }
      catch(...) // function-try-block
      {
        throw;
      }

      template<typename _Tp, __memory_policy _Policy>
      constexpr
      void
      _M_deallocate(_ptr_type& _ptr, _size_type _n_elem)
        noexcept(true)
      {
        auto _Self = _M_get_allocator();

        if constexpr(_Policy == __memory_policy::_Arena)
        {
          using _Tuple = _Tp;
          static_assert(__is_tuple_v<_Tuple>
              , "_M_deallocate() for Arena policy takes a tuple-like type.");
          return _Deallocate_hlpr<_Tuple>{}(_Self, _n_elem);
        }
        if constexpr(_Policy == __memory_policy::_Spread)
        {
          static_assert(not __is_tuple_v<_Tp>
          ,"_M_deallocate() for Spread policy takes a non tuple-like type.");
          return _Deallocate_hlpr<_Tp>{}(_Self, _n_elem);
        }
        // should not be reached.
        throw;
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
        operator()(_Alloc_base& _self, size_type _n_elem) const
        try
        {
          constexpr std::size_t _n_bytes = _n_elem * sizeof(_Tp);
          constexpr _diff_type _Align_diff = alignof(_Tp) - alignof(_byte_type);
          if constexpr(_Align_diff > 0)
          {
            /*
            * For alignment not covered by default implementation we need to be 
            * able to retrieve the original pointer before alignement correction
            * thus we store an offset factor into the original pointer.
            */  
            // adjusted number of byte for correct alignement
            constexpr std::size_t _n_bytes_adj = _n_bytes + _Align_diff;
            _ptr_type _ptr = _self.allocate(_n_bytes_adj);
            if(_ptr == _ptr_type{})
              return _ptr;
            // realign original pointer
            const _ptr_type _origin_ptr = _ptr;
            if(not std::align(alignof(_Tp), sizeof(_Tp), _ptr, _n_bytes_adj))
            {
              _self.deallocate(_ptr, _n_bytes_adj);
              // keep the original allocator exception behavior
              if constexpr(noexcept(_self.allocate({})))
                return _ptr_type{}; //nullable ptr
              else
                throw std::bad_alloc{};
            }
            /*
            * since the general rule for alignement is to use power of 2 values
            * we can prove that there is one integer x > 3 where 2^x = 8 * n
            * a multiple of 8. That is; any alignement >= 8 could be expressed
            * by an alignement of 8.
            * we will store only the factor into one byte memory.
            */ 
            _diff_type _factor = (_ptr - _origin_ptr) / alignof(_byte_type);
            if(_factor != 0 )
              _self.construct(reinterpret_cast<char*>(_ptr - 1)
                            , static_cast<char>(_factor));
            else
            { }// nothing to do, the pointer is already aligned
            return _ptr;
          }
          else
            // covered by default alignement
            return _self.allocate(_n_bytes);
        }
        catch(...)
        {// propagate exception
          throw;
        }
      };

      template<typename _Tp>
      struct _Deallocate_hlpr
      {
        [[nodiscard]]
        constexpr
        void
        operator()(_Alloc_base& _self, _ptr_type _ptr, size_type _n_elem)
          const noexcept // as per cpp standard
        {
          if(_ptr == _ptr_type{})
            return;
          /**
           * if the type's alignement is not covered by the allocator then
           * check if the pointer, to be deleted, is correctly aligned for
           * its type alignement, if not then fetch the offset factor
          */
          constexpr size_type _Align_diff = alignof(_Tp) - alignof(_byte_type);
          constexpr size_type
              _n_bytes_adj = _n_elem * sizeof(_Tp) + _Align_diff;
          if constexpr(_Align_diff > 0)
          {
            if(_ptr % alignof(_Tp) == 0) // pointer correctly aligned
              return _self.deallocate(_ptr, _n_bytes_adj);

            const auto _factor = static_cast<char>(*(_ptr - 1));
            _ptr_type _origin_ptr = _ptr - (_factor * alignof(_byte_type));
            _self.deallocate(_origin_ptr, _n_bytes_adj);
          }
          else
            _self.deallocate(_ptr, _n_bytes);
        }
      };

      template<typename... _Ts>
      struct _Allocate_hlpr<std::tuple<_Ts...>>
      {
        // helper functions
        template<typename _Tail>
        static constexpr size_type
        _M_byte_size(size_type __last_offset, size_type __n_elem) const
        {
          return __last_offset + (__n_elem * sizeof(_Tail)); 
        }

        template<typename..._Ts>
        static constexpr auto 
        _M_nbytes_and_offsets(size_type _n_elem) const ->
              std::tuple<size_type, std::array<size_type, sizeof...(_Ts)>>
        {       
          constexpr std::size_t _N = sizeof...(_Ts);
          static_assert(_N != 0, "At least one type must be provided!");

          constexpr std::array<size_type, _N> _Szof = {sizeof(_Ts)...};
          constexpr std::array<size_type, _N> _Algnof = {alignof(_Ts)...};

          // calculate total size in bytes
          std::array<size_type, _N> _M_diffs{0};

          for(size_type it = 1; it < _N; ++it)
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
          // tail type
          using _Tail = 
            typename std::tuple_element<_N - 1, std::tuple<_Ts...>>::type;

          const size_type _n_bytes = _M_byte_size<_Tail>(_M_diffs.back(), _n_elem);
          return {_n_bytes, _M_diffs};
        }

        // implementation
        [[nodiscard]]
        constexpr
        _ptr_type
        operator()(_Alloc_base& _self, size_type _n_elem) const
        try
        {
          using _1st_type = _Tuple_head<std::tuple<_Ts...>>;
          constexpr auto 
              [_n_bytes, _diffs] = _M_nbytes_and_offsets<_Ts...>(_n_elem);
          constexpr _diff_type 
            _Align_diff = alignof(_1st_type) - alignof(_byte_type);
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
            _ptr_type _ptr = _self.allocate(_n_bytes_adj);
            if(_ptr == _ptr_type{})
              return _ptr;
            // realign original pointer
            const _ptr_type _origin_ptr = _ptr;
            if( not std::align(alignof(_1st_type)
                            , sizeof(_1st_type)
                            , _ptr, _n_bytes_adj) )
            {
              _self.deallocate(_ptr, _n_bytes_adj);
              // keep the original allocator exception behavior
              if constexpr(noexcept(_self.allocate({})))
                return _ptr_type{}; //nullable ptr
              else
                throw std::bad_alloc{};
            }
            /*
            * since the general rule for alignement is to use power of 2 values
            * we can prove that there is one integer x > 3 where 2^x = 8 * n
            * a multiple of 8. That is; any alignement >= 8 could be expressed
            * by an alignement of 8.
            * we will store only the factor into one byte memory.
            */ 
            _diff_type _factor = (_ptr - _origin_ptr) / alignof(_byte_type);
            if(_factor != 0 )
              _self.construct(reinterpret_cast<char*>(_ptr - 1)
                            , static_cast<char>(_factor));
            else
            { }// nothing to do, the pointer is already aligned
            return _ptr;
          }
          else
            // covered by default alignement
            return _self.allocate(_n_bytes);
        }
        catch(...)
        {// propagate exception
          throw;
        }
      };

      template<typename..._Ts>
      struct _Deallocate_hlpr<std::tuple<_Ts...>>
      {
        constexpr 
        void 
        operator()(_Alloc_base& _self, _ptr_type _ptr, size_type _n_elem) 
        const noexcept
        {
          if(_ptr == _ptr_type{})
            return;
          using _1st_type = _Tuple_head<std::tuple<_Ts...>>;
          constexpr auto [_n_bytes, _M_diffs] 
            = _Deallocate_hlpr<void>::_M_nbytes_and_offsets<_Ts...>(_n_elem);
          constexpr _diff_type 
            _Align_diff = alignof(_1st_type) - alignof(_byte_type);
          if constexpr(_Align_diff > 0)
          {
            constexpr std::size_t _n_bytes_adj = _n_bytes + _Align_diff;
            if(_ptr % alignof(_1st_type) == 0) // pointer correctly aligned
              return _self.deallocate(_ptr, _n_bytes_adj);

            const auto _factor = static_cast<char>(*(_ptr - 1));
            _ptr_type _origin_ptr = _ptr - (_factor * alignof(_byte_type));
            _self.deallocate(_origin_ptr, _n_bytes_adj);
          }
          else
            _self.deallocate(_ptr, _n_bytes);          
        }
      };
    };

    /**
     * 
    */
    template<typename _Alloc, typename... _Ts>
    struct _couplvecs_Impl: _Alloc_base<_Alloc>, _Ptrs_array_base<_Ts...>
    {
      using _allocator_type = _Alloc_base<_Alloc>;

      template<typename _Tp>
      struct _iterator;

      template<typename _Tp>
      struct _const_iterator;
    };
  } // namespace __detail

  
  // partial specialization
  // std::pair is a special case of std::tuple
  template<typename..._Ts, typename _Alloc>
  class couplvecs<std::tuple<_Ts...>, _Alloc>
    : private __detail::_couplvecs_Impl<_Alloc, _Ts...>
  {
    // data members.
    std::size_t _m_max_arena;

    /**
     * typedefs
    */
   private:
    using __base_type  = __detail::_couplvecs_Impl<_Alloc, _Ts...>;
    using __base_alloc = typename __base_type::_allocator_type;
    
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

    
   public:
    // Member functions

    /**
     * @tparam _Mem_max_size : 
     *         maximum memory allowed, for Arena allocation policy, after which
     *         the implementation switchs to allocating each buffer separratly.
    */
    template<std::size_t _Mem_max_size>
    couplvecs() noexcept
    : _M_data(__detail::_couplvecs_arena_Impl<_Alloc, _Ts...>)
    , _M_mem_sz(_max_mem_size)
    { }

    // Element access
    [[deprecated("NotImplemented")]]
    constexpr reference
    at(size_type _Idx)
    { }

    [[deprecated("NotImplemented")]]
    constexpr const_reference
    at(size_type _Idx) const
    { }

    [[deprecated("NotImplemented")]]
    constexpr reference
    operator[](size_type _Idx)
    { }

    [[deprecated("NotImplemented")]]
    constexpr const_reference
    operator[](size_type _Idx) const
    { }

    [[deprecated("NotImplemented")]]
    constexpr reference
    front() noexcept
    { }

    [[deprecated("NotImplemented")]]
    constexpr const_reference
    front() const noexcept
    { }

    [[deprecated("NotImplemented")]]
    constexpr reference
    back() noexcept
    { }

    [[deprecated("NotImplemented")]]
    constexpr const_reference
    back() const noexcept
    { }

    [[deprecated("NotImplemented")]]
    constexpr pointer
    data() noexcept
    { }

    [[deprecated("NotImplemented")]]
    constexpr const_pointer
    data() const noexcept
    { }

    // Iterators
    [[deprecated("NotImplemented")]]
    constexpr iterator
    begin() noexcept
    { }

    [[deprecated("NotImplemented")]]
    constexpr const_iterator
    begin() const noexcept
    { }

    [[deprecated("NotImplemented")]]
    constexpr const_iterator
    cbegin() const noexcept
    { }

    [[deprecated("NotImplemented")]]
    constexpr iterator
    end() noexcept
    { }

    [[deprecated("NotImplemented")]]
    constexpr const_iterator
    end() const noexcept
    { }

    [[deprecated("NotImplemented")]]
    constexpr const_iterator
    cend() const noexcept
    { }

    [[deprecated("NotImplemented")]]
    constexpr reverse_iterator
    rbegin() noexcept
    { }

    [[deprecated("NotImplemented")]]
    constexpr const_reverse_iterator
    rbegin() const noexcept
    { }

    [[deprecated("NotImplemented")]]
    constexpr const_reverse_iterator
    crbegin() const noexcept
    { }

    [[deprecated("NotImplemented")]]
    constexpr reverse_iterator
    rend() noexcept
    { }

    [[deprecated("NotImplemented")]]
    constexpr const_reverse_iterator
    rend() const noexcept
    { }

    [[deprecated("NotImplemented")]]
    constexpr const_reverse_iterator
    crend() const noexcept
    { }

    // Capacity
    [[deprecated("NotImplemented")]]
    constexpr bool
    empty() const noexcept
    { }

    [[deprecated("NotImplemented")]]
    constexpr size_type
    size() const noexcept
    { }

    [[deprecated("NotImplemented")]]
    constexpr size_type
    max_size() const noexcept
    { }

    [[deprecated("NotImplemented")]]
    constexpr void
    reserve(size_type _n_elem)
    { }

    [[deprecated("NotImplemented")]]
    constexpr size_type
    capacity() const noexcept
    { }

    [[deprecated("NotImplemented")]]
    constexpr bool
    squeeze() noexcept
    { }
    // Modifiers
    [[deprecated("NotImplemented")]]
    constexpr void
    clear() noexcept
    { }

    [[deprecated("NotImplemented")]]
    constexpr void
    insert()
    { }

    [[deprecated("NotImplemented")]]
    constexpr void
    insert_range()
    { }

    [[deprecated("NotImplemented")]]
    constexpr void
    emplace()
    { }

    [[deprecated("NotImplemented")]]
    constexpr void
    erase()
    { }
    
    [[deprecated("NotImplemented")]]
    void push_back(const_reference _val)
    { }

    [[deprecated("NotImplemented")]]
    void push_back(value_type&& _rval)
    { }

    [[deprecated("NotImplemented")]]
    constexpr void
    append_range()
    { }

    [[deprecated("NotImplemented")]]
    constexpr void
    pop_back()
    { }

    [[deprecated("NotImplemented")]]
    constexpr void
    resize()
    { }

    [[deprecated("NotImplemented")]]
    constexpr void
    swap()
    { }

    // Non-member functions
    [[deprecated("NotImplemented")]]
    friend bool operator==(const couplvecs& lft, const couplvecs& rht)
    { }

    [[deprecated("NotImplemented")]]
    friend bool operator!=(const couplvecs& lft, const couplvecs& rht)
    { return not (lft == rht); }

    [[deprecated("NotImplemented")]]
    friend std::swap(couplvecs& lft, couplvecs rht)
    { }
    
  };


}// namespace stl
#endif // _COUPLED_VECTORS_H