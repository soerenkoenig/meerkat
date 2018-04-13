//
//           .___.
//           {o,o}
//          ./)_)
//      owl --"-"---
//
//  Copyright © 2017 Sören König. All rights reserved.
//

#pragma once
#include <functional>
#include <stack>
#include <cassert>
#include <vector>
#include <chrono>
#include "owl/utils/range_algorithm.hpp"

namespace owl
{
  namespace utils
  {
    class progress
    {
    public:
    
      progress(std::uint64_t num_steps)
        : num_steps_(num_steps)
      {
        start_ = std::chrono::steady_clock::now();
        if(current())
          add_child_to_current(this);
      }
    
      bool is_cancellable() const
      {
        return is_leaf() ? is_cancellable_ : owl::utils::all_of(children_,
          [](const auto& c) { return c->is_cancellable();});
      }
    
      void cancel()
      {
        assert(is_cancellable());
        for(auto& c: children_)
          c->cancel();
        is_canceled_ = true;
      }
    
       bool is_pausable() const
      {
        return is_leaf() ? is_pausable_ : owl::utils::all_of(children_,
          [](const auto& c) { return c->is_pausable();});
      }
  
      void pause()
      {
        assert(is_pausable());
        for(auto& c: children_)
          c->pause();
        is_paused_ = true;
      }
  
      void resume()
      {
        assert(is_pausable());
        for(auto& c: children_)
          c->pause();
        is_paused_ = false;
      }
  
      bool is_indeterminant() const
      {
        return num_steps_ == 0 && num_steps_completed_ == 0;
      }
  
      double fraction_completed() const
      {
        if(num_steps_ == 0)
          return 0;
      
        auto fc = static_cast<double>(num_steps_completed_) / num_steps_;
        for(const auto& child : children_)
        {
          if(child->num_steps_completed_ != child->num_steps_)
            fc += child->fraction_completed() * static_cast<double>(child->num_steps_parent_)/num_steps_;
        }
        if(fc > 1)
          return fc;
        return fc;
      }
    
      auto estimated_time_remaining() const
      {
        if(num_steps_completed_ == num_steps_)
          return 0.0 * (start_ - start_);
        auto now =  std::chrono::steady_clock::now();
        return (now - start_) * (1.0 - fraction_completed())/fraction_completed();
      }
    
      void make_current(std::uint64_t num_steps_pending)
      {
        if(current() != this)
          current_progress_.push(current_info{this, num_steps_pending});
      }
  
      void resign_current()
      {
        assert(current() == this);
        if(children_.size() == 0)
          num_steps_completed_ += current_progress_.top().num_steps_pending;
        current_progress_.pop();
        children_.clear();
      }
  
      void step(std::uint64_t n = 1, bool notify_change = true)
      {
        num_steps_completed_ += n;
        assert(num_steps_completed_ <= num_steps_);
        if(parent_ != nullptr && num_steps_completed_ == num_steps_)
          parent_->step(num_steps_parent_, false);
        if(notify_change)
          notify_changed();
      }
    
      void add_child(progress* child, std::uint64_t num_steps_pending)
      {
        children_.push_back(child);
        child->parent_ = this;
        child->num_steps_parent_ = num_steps_pending;
      }
  
      std::function<void()> on_changed;
    
    private:
      bool is_leaf() const
      {
        return parent_ == nullptr;
      }
    
      void notify_changed()
      {
        if(on_changed)
          on_changed();
        if(parent_)
          parent_->notify_changed();
      }
    
      /*void balance(std::uint64_t num_steps_pending)
      {
        for(std::size_t i = 0; i < children_.size(); ++i)
        {
          children_[i]->num_steps_parent_ = (i + 1) * num_steps_pending / children_.size();
        }
      }*/
      
      static progress* current()
      {
        if(progress::current_progress_.empty())
          return nullptr;
        return progress::current_progress_.top().current;
      }
    
      static void add_child_to_current(progress* child)
      {
        assert(current() != nullptr);
        
        current()->add_child(child, current_progress_.top().num_steps_pending);
        //current()->balance(current_progress_.top().num_steps_pending);
      }
    
      struct current_info
      {
        progress* current;
        std::size_t num_steps_pending;
      };
    
      static thread_local std::stack<current_info> current_progress_;
  
      progress* parent_ = nullptr;
      std::atomic<std::uint64_t> num_steps_parent_ = 0;

      std::vector<progress*> children_;
      std::atomic<std::uint64_t> num_steps_ = 0;
      std::atomic<std::uint64_t> num_steps_completed_ = 0;
  
      bool is_canceled_ = false;
      bool is_cancellable_ = false;
    
      bool is_paused_ = false;
      bool is_pausable_ = false;
      std::chrono::steady_clock::time_point start_;
  
    };
  }
}
