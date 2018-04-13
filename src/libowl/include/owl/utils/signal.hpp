
#pragma once

#include <functional>
#include <memory>
#include <map>
#include <vector>
#include <iostream>

#include "owl/export.hpp"
#include "owl/utils/non_copyable.hpp"
#include "owl/utils/handle.hpp"

namespace owl
{
 namespace utils
 {

    template<typename Result>
    struct combiner_last
    {
      using result_type = Result;
      using combined_result_type = Result;

      explicit combiner_last() : result_()
      {
      }

      bool operator()(result_type r)
      {
        result_ = r;
        return true;
      }

      combined_result_type get() const
      {
        return result_;
      }
      
    private:
      combined_result_type result_;
    };
 
    template<typename Result>
    struct combiner_default : combiner_last<Result>
    {
      using result_type = typename combiner_last<Result>::result_type;
      using combined_result_type =typename combiner_last<Result>::combined_result_type;
    };


    template<>
    struct OWL_API combiner_default<void>
    {
      using result_type = void;
      using combined_result_type = void;

      void get() const {}
      bool operator()(void)  { return true; }
    };
 
    template<typename Result>
    struct combiner_vector
    {
      using result_type = Result;
      using combined_result_type = std::vector<Result>;

      const combined_result_type& get() const
      {
        return result_;
      }

      bool operator()(result_type r)
      {
        result_.push_back(r);
        return true;
      }
  
    private:
      combined_result_type result_;
    };
 
    struct OWL_API combiner_and
    {
      using result_type = bool;
      using combined_result_type = bool;

      explicit combiner_and() {}

      const combined_result_type& get() const  { return result_; }

      combined_result_type operator()(result_type r)
      {
        result_ = ( result_ && r );
        return true;
      }

    private:
      combined_result_type result_ = true;
    };
 
    struct OWL_API combiner_or
    {
      using result_type = bool;
      using combined_result_type = bool;

      explicit combiner_or() {}

      const combined_result_type& get() const
      {
        return result_;
      }
        
      combined_result_type operator()(result_type r)
      {
        result_ = ( result_ || r );
        return true;
      }

    private:
      combined_result_type result_ = false;
    };
 
    template<typename Result>
    struct combiner_bitwise_and
    {
      using result_type = Result;
      using combined_result_type = Result;

      explicit combiner_bitwise_and() {}

      const combined_result_type& get() const { return result_; }

      combined_result_type operator()(result_type r)
      {
        if(first_)
        {
          result_ = r;
          first_ = false;
          return true;
        }
      
        result_ &= r ;
        return true;
      }

    private:
      combined_result_type result_ = 0;
      bool first_ = true;
    };
 
 
    template<typename Result>
    struct combiner_until_false
    {
      using result_type = Result;
      using combined_result_type = Result;

      explicit combiner_until_false() : result_() {}

      const combined_result_type& get() const { return result_; }

      bool operator()(result_type r)
      {
        result_ = r;
        return result_ ? true : false;
      }

    private:
      combined_result_type result_;
    };
 
    template<typename Result>
    struct combiner_while_false
    {
      using result_type = Result;
      using combined_result_type = Result;

      explicit combiner_while_false() : result_() {}

      const combined_result_type& get() const  { return result_; }

      bool operator()( result_type r )
      {
        result_ = r;
        return  result_ ? false : true ;
      }

    private:
      combined_result_type result_;
    };
   
    struct OWL_API callback_tag{};
    using callback_handle = owl::utils::handle<callback_tag>;
 
    namespace detail
    {
      class OWL_API signal_base : non_copyable
      {
        virtual bool disconnect(callback_handle handle, int priority) = 0;

        friend struct disconnector;
      };
  
      struct disconnector : private non_copyable
      {
        disconnector(signal_base *signal)
          : owning_signal_{signal}
        {
          
        }
    
        bool disconnect(callback_handle handle, std::int32_t priority)
        {
          return owning_signal_->disconnect(handle, priority);
        }

      private:
        signal_base*  owning_signal_;
      };
    }
 
   
    class OWL_API connection
    {
    public:
     
      connection()
        : priority_(0)
      {
      }
      
      connection(const std::shared_ptr<detail::disconnector>& discon, callback_handle handle, int priority = 0)
        : handle_(handle)
        , priority_(priority)
        , disconnector_(discon)
      {
      }
      
      connection(const connection &other)
        : disconnector_(other.disconnector_)
        , handle_(other.handle_)
        , priority_(other.priority_)
      {
      }
      
      connection(connection &&other)
        : disconnector_(std::move( other.disconnector_))
        , handle_(std::move( other.handle_))
        , priority_(std::move( other.priority_))
      {
      }
      
      connection& operator=(const connection &other)
      {
        disconnector_ = other.disconnector_;
        handle_ = other.handle_;
        priority_ = other.priority_;
        
        return *this;
      }
      
      connection& operator=(connection &&other)
      {
        disconnector_ = std::move(other.disconnector_);
        handle_ = std::move(other.handle_);
        priority_ = std::move(other.priority_);
        
        // TODO: remove once all compilers support C++14
        other.disconnector_.reset();
        
        return *this;
      }
      
      ~connection()
      {
        disconnect();
      }
      
      bool disconnect()
      {
        if(!handle_.is_valid())
          return true;
        
        const auto discon = disconnector_.lock();
        if(!discon)
          return true;
        
        if(!discon->disconnect(handle_, priority_))
          return false;
        handle_.invalidate();
        return true;
      }
      
      bool is_connected() const
      {
        return !disconnector_.expired() && handle_.is_valid();
      }

      
    private:
      std::weak_ptr<detail::disconnector> disconnector_;
      callback_handle handle_;
      int priority_;
    };
   
   class OWL_API scoped_connection : public connection, private non_copyable
   {
   public:
     
     scoped_connection()
     {
     }
     
     scoped_connection(scoped_connection &&other)
       : connection( std::move(other))
     {
     }
     
     scoped_connection(connection &&other)
      : connection(std::move( other))
     {
     }
     
     ~scoped_connection()
     {
       disconnect();
     }
     
     scoped_connection& operator=(scoped_connection &&other)
     {
       disconnect();
       connection::operator=(std::move(other));
       return *this;
     }
     
   };
   
   class OWL_API connection_list : private non_copyable
   {
   public:
     ~connection_list()
     {
       clear();
     }
     
     void add(connection &&target)
     {
       connections_.emplace_back(std::move(target));
     }
     
     void clear()
     {
       for(auto &conn : connections_)
         conn.disconnect();
       
       connections_.clear();
     }
     
     void operator+=(connection &&target)
     {
       add(std::move( target));
     }
     
   private:
     std::vector<connection> connections_;
   };

   
    namespace detail
    {
      template<typename, typename> struct  signal_invoker;
    
      template<class Combiner, class Result, class... Args>
      struct signal_invoker<Combiner, Result ( Args... )> : public signal_base
      {
        bool invoke(Combiner &combiner, const std::function<Result (Args...)> &callback, Args... args)
        {
          return combiner( callback( args... ) );
        }
      };

      template<class Combiner, class... Args>
      struct signal_invoker<Combiner, void(Args...)> : public signal_base
      {
        bool invoke(Combiner &combiner, const std::function<void(Args...)> &callback, Args... args)
        {
          callback(args...);
          return combiner();
        }
      };
    
      template <typename, typename> class signal_proto;
 
      template <typename Result, typename... Args, typename Combiner>
      class signal_proto<Result(Args...), Combiner> : public signal_invoker<Combiner, Result(Args...)>
      {
      public:
        using callback_type = std::function<Result(Args...)>;
        using result_type = Result;
        using combined_result_type = typename Combiner::combined_result_type;
        
        signal_proto()
          : disconnector_(std::make_shared<disconnector>(this))
        {
          
        }
  
        template <typename... ArgsIn>
        combined_result_type operator()(ArgsIn&&... args)
        {
          Combiner combiner;
          for(auto&& priority_group : callbacks_)
          {
            for(auto&& callback : priority_group.second)
            {
              if(!callback.enabled_ || !callback.func_)
                continue;
            
              callback.enabled_ = false;
              bool continue_emit = this->invoke(combiner, callback.func_, std::forward<ArgsIn>(args)...);
              callback.enabled_ = true;
            
              if(!continue_emit)
                break;
            }
          }
          return combiner.get();
        }
      
        connection connect(callback_type fn, int priority = 0)
        {
          //todo connect during emit is not safe
          const auto handle = next_handle_++;
          callbacks_[priority].emplace_back(callback_entry{handle, std::move(fn), true});
          return connection{ disconnector_, handle, priority };
        }
        
        bool disconnect(callback_handle handle, int priority = 0) override
        {
          //todo disconnect during emit is not safe
          auto git = callbacks_.find(priority);
          if(git == callbacks_.end())
            return false;
          auto it = std::find_if(git->second.begin(), git->second.end(),[&](auto& c) { return c.handle_ == handle;});
          if( it == git->second.end())
            return false;
          git->second.erase(it);
          if(git->second.size() == 0)
            callbacks_.erase(git);
          return true;
        }
        
        std::size_t num_connections() const
        {
          std::size_t count = 0;
          for(auto&& cg: callbacks_)
            count += cg.second.size();
          return count;
        }
    
      private:
        struct callback_entry
        {
          callback_handle handle_;
          callback_type func_;
          bool enabled_;
        };
    
        callback_handle next_handle_{0};
        std::map<std::int32_t, std::vector<callback_entry>, std::greater<int>> callbacks_;
        std::shared_ptr<disconnector> disconnector_;
      };
    }
 
    template <typename Signature, typename Combiner = combiner_default<typename std::function<Signature>::result_type>>
    class signal : public detail::signal_proto<Signature, Combiner>
    {
    };
   
    template<class Instance, class Class, class Result, class... Args>
    std::function<Result (Args...)> slot(Instance &object, Result (Class::*method)(Args...))
    {
      return [&object, method] (Args... args)  { return ( object .* method )( args... ); };
    }

    template<class Class, class Result, class... Args>
    std::function<Result (Args...)> slot(Class *object, Result ( Class::*method )(Args...))
    {
      return [object, method] (Args... args)  { return ( object ->* method )( args... ); };
    }
  }
}

