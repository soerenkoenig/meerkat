//
//           .___.
//           {o,o}
//          ./)_)
//      owl --"-"---
//
//  Copyright © 2017 Sören König. All rights reserved.
//

#pragma once

#include <vector>
#include <cassert>
#include <algorithm>
#include <numeric>
#include "owl/export.hpp"
#include "owl/utils/handle.hpp"
#include "owl/utils/count_iterator.hpp"


namespace owl
{
  namespace utils
  {
  
    namespace detail
    {
      struct component_tag{};
      using component_handle = owl::utils::handle<component_tag>;
   
      struct element_tag{};
      using element_handle = owl::utils::handle<element_tag>;
    }

    template <typename ElementHandle = detail::element_handle,
      typename ComponentHandle = detail::component_handle>
    class connected_components
    {
      struct node
      {
        std::size_t parent;
        std::size_t count;
      };
    
    public:
    
      using element_handle = ElementHandle;
      using component_handle = ComponentHandle;
    
    
      struct const_iterator
      {
        using iterator_category = std::forward_iterator_tag;
        using value_type = component_handle;
        using pointer = void;
        using reference = void;
        using size_type =  std::size_t;
        using difference_type = std::ptrdiff_t;

        const_iterator() = default;
      
        const_iterator(const connected_components* comp, std::size_t current)
          : connected_components_(comp)
          , current_(current)
        {}
      
        component_handle operator*() const
        {
          return component_handle(current_);
        }
  
        const_iterator& operator++()
        {
          current_++;
          while(current_ < connected_components_->num_elements() &&
                current_ != connected_components_->nodes_[current_].parent)
            current_++;

          return *this;
        }
      
        const_iterator operator++(int)
        {
          const_iterator temp = *this;
          ++*this;
          return temp;
        }
      
        bool operator==(const const_iterator& other) const
        {
          return current_ == other.current_ && connected_components_ == other.connected_components_;
        }
     
        bool operator!=(const const_iterator& other) const
        {
          return current_ != other.current_ || connected_components_ != other.connected_components_;
        }

      private:
        const connected_components* connected_components_ = nullptr;
        std::size_t current_;
      };
    
      connected_components() = default;
   
      connected_components(std::size_t N) : nodes_(N), num_components_(N)
      {
        for(std::size_t i = 0; i < N; ++i)
        {
          nodes_[i].parent = i;
          nodes_[i].count = 1;
        }
      }
    
      std::size_t num_components() const
      {
        return num_components_;
      }

      std::size_t num_elements() const
      {
        return nodes_.size();
      }
    
      std::size_t num_elements(component_handle c) const
      {
       assert(c.index() < num_elements());
       return find(c).count;
      }

      const_iterator begin() const
      {
       std::size_t x = 0;
       while(x != nodes_[x].parent)
         x++;
       return const_iterator(this, x);
      }
    
      const_iterator cbegin() const
      {
         return begin();
      }
    
      const_iterator end() const
      {
       return const_iterator(this, num_elements());
      }
    
      const_iterator cend() const
      {
       return end();
      }
    
      element_handle element(std::size_t index) const
      {
        return index < num_elements() ? element_handle(index) : element_handle();
      }
    
      component_handle component(element_handle e) const
      {
        std::size_t x = e.index();
        while(x != nodes_[x].parent)
        {
          nodes_[x].parent = nodes_[nodes_[x].parent].parent;
          x = nodes_[x].parent;
        }
        return component_handle(x);
      }

      bool is_connected(element_handle p, element_handle q) const
      {
        return find(p).parent == find(q).parent;
      }
    
      component_handle connect(element_handle p, element_handle q)
      {
        return connect(component(p), component(q));
      }
    
      component_handle connect(element_handle p, component_handle q)
      {
        return connect(component(p), q);
      }
    
      component_handle connect(component_handle p, element_handle q)
      {
        return connect(p, component(q));
      }
    
      component_handle connect(component_handle p, component_handle q)
      {
        auto& ni = find(p);
        auto& nj = find(q);
        if (ni.parent == nj.parent)
          return component_handle(ni.parent);

        if(ni.count < nj.count)
        {
          ni.parent = nj.parent;
          nj.count += ni.count;
        }
        else
        {
          nj.parent = ni.parent;
          ni.count += nj.count;
        }
        num_components_--;
        return component_handle(ni.parent);
      }

  private:
      node& find(element_handle e)
      {
        return find(component(e));
      }
    
      node& find(component_handle c)
      {
        return nodes_[c.index()];
      }
    
      const node& find(element_handle e) const
      {
        return find(component(e));
      }
    
      const node& find(component_handle c) const
      {
        return nodes_[c.index()];
      }
    
      mutable std::vector<node> nodes_;
      std::size_t num_components_;
    };
  }
}
