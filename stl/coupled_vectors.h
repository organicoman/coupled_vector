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

    /// @brief _Alloc_base: base class to manage allocation of the coupled
    ///        buffers.
    ///        Two policies could be used; either allocate a contiguous arena
    ///        to hold all buffers, Or allocate the necessary size for each
    ///        buffer separatly. The minimum size to switch from arena to
    ///        individual buffers, is user given.
    ///        The Arena policy is good for locality and cache hits. But
    ///        could raise std::bad_array_new_length exception if the system
    ///        cannot provide the requested size.
    /// @tparam _Alloc: allocator type. 
    ///         For Arena allocation policy, we rebind
    ///         this allocator to std::byte type, to reserve a contiguous chunck
    ///         of memory, then we assign each coupled buffer its appropriate
    ///         partition of memory from this arena (type alignment 
    ///         and padding must be respected between each buffer's partition
    ///         when calculating each partition size and begin pointer).
    ///         For spread buffers allocation policy, we allocate each 
    ///         buffer array separately, respecting alignment.
    /// @tparam _Policy: __memory_policy enum value. 
    ///         _Arena: for allocating an Arena greater or equal to the sum of 
    ///         buffers sizes. 
    ///         _Spread: for individual allocations by at least buffer size.
    
    template<typename _Alloc>
    struct _Alloc_base
    : public _rebind_Allocator<_Alloc, aligned_byte<alignof(void*)>> 
    {
      private:
      enum __memory_policy { _Spread  = false, _Arena = true};
      enum __Enum { _Stateless, _Statefull };

      public:
      using _byte_type = aligned_byte<alignof(void*)>;
      using _base_type = _rebind_Allocator<_Alloc, _byte_type>
      using _alloc_traits = std::allocator_traits<_base_type>;
      using _value_type = typename _alloc_traits::value_type;
      using _ptr_type = typename _alloc_traits::pointer;
      using _size_type = typename _alloc_traits::size_type;
      using _diff_type = typename _alloc_traits::difference_type;

      /**
       * concret class implementation
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
          return _Allocate_hlpr<_Tuplel>{}(_Self, _n_elem);
        }
        else
        {
          return _Allocate_hlpr<_Tp>{}(_Self, _n_elem);
        }
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
          // stateless original allocator
          return _Deallocate_hlpr<_Tuple>{}(_Self, _n_elem);
        }
        else
        {
          return _Deallocate_hlpr<_Tp>{}(_Self, _n_elem);
        }
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
        operator()(_Alloc_base& _self, size_type _n_elem)
        try
        {
          constexpr std::size_t _n_bytes = _n_elem * sizeof(_Tp);
          constexpr _diff_type _Align_diff = alignof(_Tp) - alignof(_byte_type);
          if constexpr(_Align_diff > 0)
          {
            /*
            * For alignment not cover by default implementation we need to be 
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
              _self.deallocate(_ptr, _n_bytes);
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
            _diff_type _factor = (_ptr - _origin_ptr) / 8;
            if(_factor != 0 )
              _self.construct(reinterpret_cast<char*>(_ptr - 1)
                            , static_cast<char>(_factor));
            else
            { }// nothing to do, the pointer is already aligned
            return _ptr;
          }
          else
            return _self.allocate(_n_bytes);
        }
        catch(...)
        {// propagate exception
          throw;
        }
      };

      template<typename... _Ts, __Enum _Alloc_type>
      struct _Allocate_hlpr<std::tuple<_Ts...>, _Alloc_type>
      {
        // helper functions
        template<typename _Tail>
        constexpr size_type
        _M_byte_size(size_type __last_offset, size_type __n_elem) const
        {
          return __last_offset + (__n_elem * sizeof(_Tail)); 
        }

        template<typename..._Ts>
        constexpr auto 
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
            auto mod = _Algnof[it] < alignof(_byte_type) ? alignof(_byte_type) : _Algnof[it];
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
        operator()(_Alloc_base& _self, size_type _n_elem)
        try
        {
          using _1st_type = _Tuple_head<std::tuple<_Ts...>>;
          constexpr std::size_t _n_bytes = _n_elem * sizeof(_1st_type);
          constexpr _diff_type 
            _Align_diff = alignof(_1st_type) - alignof(_byte_type);
        }
        catch(...)
        {// propagate exception
          throw;
        }
      };

      template<__memory_policy _Policy>
      struct _Alloc_base_Impl
      {
        

        template<typename..._Ts>
        struct _M_allocate_hlpr
        {
          [[nodiscard]]
          constexpr 
          pointer 
          operator()(size_type _n_elem) const
          {
            auto [_n_bytes, _M_diffs] = _M_nbytes_and_offsets<_Ts...>(_n_elem);
            return _alloc_traits::allocate(_M_get_allocator(), _n_bytes);
          }
        };

        template<typename..._Ts>
        struct _M_allocate_hlpr<std::tuple<_Ts...>>
        {
          [[nodiscard]]
          constexpr 
          pointer 
          operator()(size_type _n_elem) const
          {
            auto [_n_bytes, _M_diffs] = _M_nbytes_and_offsets<_Ts...>(_n_elem);
            return _alloc_traits::allocate(_M_get_allocator(), _n_bytes);
          }
        };


        template<typename..._Ts>
        struct _M_deallocate_hlpr
        {
          constexpr 
          void 
          operator()(pointer _ptr, size_type _n_elem) const noexcept
          {
            auto [_n_bytes, _M_diffs] = 
              _M_nbytes_and_offsets<_Ts...>(_n_elem);
            return _alloc_traits::deallocate(_M_get_allocator(),_ptr, _n_bytes);~
          }
        };

        template<typename..._Ts>
        struct _M_deallocate_hlpr<std::tuple<_Ts...>>
        {
          constexpr
          void
          operator()(pointer _ptr, size_type _n_elem) const noexcept
          {
            auto [_n_bytes, _M_diffs] = 
              _M_nbytes_and_offsets<_Ts...>(_n_elem);
            return _alloc_traits::deallocate(_M_get_allocator(), _ptr, _n_bytes);
          }
        };
      };
    };

    // _Alloc_base : implementation for Spread-allocation policy
    template<typename _Alloc>
    struct _Alloc_base<_Alloc, __memory_policy::_Spread> 
    : public _rebind_Allocator<_Alloc, alignof(void*)>
    {
      using _byte_type = aligned_byte<alignof(void*)>;
      using _base_type = _rebind_Allocator<_Alloc, alignof(void*)>;
      using _alloc_traits = std::allocator_traits<_base_type>;
      using pointer = typename _alloc_traits::pointer;
      using size_type = typename _alloc_traits::size_type;

      _Alloc_base& 
      _M_get_allocator() noexcept
      { return *this; }

      const _Alloc_base& 
      _M_get_allocator() const noexcept
      { return *this; }
      
      template<typename _Tp>
      [[nodiscard]] 
      constexpr pointer 
      _M_allocate(size_type _n_elem)
      {
        constexpr typename _alloc_traits::difference_type _Align_diff 
          = alignof(_Tp) - alignof(_byte_type);

        constexpr size_type _n_bytes = _n_elem * sizeof(_Tp);

        // case Where original allocator is stateless
        if constexpr (std::allocator_traits<_Alloc>::is_always_equal{})
        {
          auto __other = _rebind_Allocator<_Alloc, alignof(_Tp)>{};
          return _alloc_traits::allocate(__other, _n_bytes);
        }
        else if constexpr (_Align_diff <= 0)
        {
          try
          {
            pointer _M_ptr = 
              _alloc_traits::allocate(_M_get_allocator(), _n_bytes);
            return _M_ptr;
          }
          catch(...)
          // propagate exception
            throw;
        }
          
        //  For any other specific alignement, we need to re-adjust the
        // requested byte size, and keep track of the old pointer
        else 
        {
          size_type _n_bytes_ext = _n_bytes + _Align_diff;
          const auto _old_n = _n_bytes_ext;
          try
          {
            pointer _M_ptr =
               _alloc_traits::allocate(_M_get_allocator(), _n_bytes_ext);
            if(_M_ptr == pointer{})
              return pointer{};
            // adjust pointer according to alignement
            const pointer _M_old_ptr = _M_ptr;
            if(not std::align(alignof(_Tp), sizeof(_Tp)
                        , static_cast<void*&>(_M_ptr), _n_bytes_ext))
            {
              _alloc_traits::deallocate(_M_get_allocator(), _M_ptr, _n_bytes_ext);
              // keep the behavior of the base allocator class
              // if it throws then throw.
              if constexpr (noexcept(_alloc_traits::allocate({},{})))
                return pointer{};
              else
                throw std::bad_alloc();
            }
            // since the general rule for alignement is to use power of 2 values
            // we can prove that there is one integer x > 3 where 2^x = 8 * n
            // a multiple of 8. That is any alignement >= 8 could be expressed
            // by an alignement of 8.
            // we will store only the factor into one byte memory.
            ptrdiff_t _M_factor = (_M_ptr - _M_old_ptr) / 8;
            if(_M_factor != 0 )
              _alloc_traits::construct(_M_get_allocator(), (_M_ptr - 1)
                                    , static_cast<char>(_M_factor));
            else // move forward the new pointer
              _alloc_traits::construct(_M_get_allocator(), 
              ((_M_ptr+=_Align_diff) - 1), static_cast<char>(_M_factor));

            return _M_ptr;
          }
          catch(...)
          // propagate exception
            throw;          
        }        
      }

      //  Deallocation function, will check if the the type has greater
      // alignement requirement than alignof(void*), if true, we read the
      // offset value from the byte located before the pointer argument
      // then we calculate the pointer difference to get the original
      // pointer returned by the allocation function, then we deallocate
      // using that pointer value.
      template<typename _Tp>
      constexpr 
      void
      _M_deallocate(pointer _ptr, size_type _n_elem)
        noexcept(noexcept(_alloc_traits::deallocate({}, {}, {})))
      {
        if(_ptr == pointer{})
          return;
        
        constexpr size_type _Align_diff = alignof(_Tp) - alignof(_byte_type);
        constexpr size_type _n_bytes = _n_elem * sizeof(_Tp);
        // original alloator is stateless.
        if constexpr (std::allocator_traits<_Alloc>::is_always_equal{})
        {
          auto& __other = _rebind_Allocator<_Alloc, alignof(_Tp)>;
          return _alloc_traits::deallocate(__other, _ptr, _n_bytes);
        }
        else if constexpr(_Align_diff <= 0)
          return _alloc_traits::deallocate(_M_get_allocator(), _ptr, _n_bytes);
        else
        {          
          // read the stored value
          const size_type _n_bytes_ext = _n_elem * sizeof(_Tp) + _Align_diff;
          const auto _M_factor = static_cast<char>(*(_ptr - 1));
          pointer _M_ptr = _ptr - (_M_factor * 8);
          _alloc_traits::deallocate(_M_get_allocator(), _M_ptr, _n_bytes_ext);
          return;
        }
      }
    };

    /**
     * 
    */
    template<typename _Base_alloc, typename... _Ts>
    struct _couplvecs_Impl;

    template<__memory_policy _Policy, typename _Alloc, typename... _Ts>
    struct _couplvecs_Impl<_Alloc_base<_Alloc, _Policy>, _Ts...>
    : public _Alloc_base<_Alloc, _Policy>
    , protected _Ptrs_array_base<_Ts...>
    {

    };  

    template<typename _Alloc, typename ..._Ts>
    using _couplvecs_arena_Impl = 
      _couplvecs_Impl<_Alloc_base<_Alloc, true>, _Ts...>;

    template<typename _Alloc, typename ..._Ts>
    using _couplvecs_spread_Impl = 
      _couplvecs_Impl<_Alloc_base<_Alloc, false>, _Ts...>;

    template<typename _Alloc, typename ..._Ts>
    using _couplvecs_variant_t = 
      std::variant<_couplvecs_arena_Impl<_Alloc, _Ts...>
                 , _couplvecs_spread_Impl<_Alloc, _Ts...>>;
  } // namespace __detail

  
  // partial specialization
  // std::pair is a special case of std::tuple
  template<typename..._Ts, typename _Alloc>
  class couplvecs<std::tuple<_Ts...>, _Alloc>
  {
    _couplvecs_variant_t _M_data;
    std::size_t          _M_mem_sz;

    /**
     * typedefs
    */
    public:
    using value_type = std::tuple<_Ts...>;


    // Ctors
    public:
    template<std::size_t _max_mem_size>
    couplvecs() noexcept
    : _M_data(__detail::_couplvecs_arena_Impl<_Alloc, _Ts...>)
    , _M_mem_sz(_max_mem_size)
    { }

  };


}// namespace stl
#endif // _COUPLED_VECTORS_H