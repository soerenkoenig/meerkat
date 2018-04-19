//
//           .___.
//           {o,o}
//          ./)_)
//      owl --"-"---
//
//  Copyright (c) 2018 Sören König. All rights reserved.
//

#pragma once

#include <stack>
#include <memory>
#include <queue>
#include <limits>

#include "owl/optional.hpp"
#include "owl/math/interval.hpp"
#include "owl/math/primitive_traits.hpp"
#include "owl/math/point_utils.hpp"


namespace owl
{
  namespace math
  {
    template<typename Scalar, std::size_t Dimension>
    vector <Scalar, Dimension> reference_point(const vector <Scalar, Dimension> &v)
    {
      return v;
    };

    template<typename Scalar, std::size_t Dimension>
    const vector <Scalar, Dimension> &
    closest_point(const vector <Scalar, Dimension> &prim, const vector <Scalar, Dimension> &q)
    {
      return prim;
    };

    namespace detail
    {

      struct default_deref
      {
        template<typename T>
        auto operator()(T&& p) const
        {
          return std::forward<T>(p);
        }
      };
    }

    template<typename Primitive,
      typename Deref = detail::default_deref,
      typename Vector = typename vector_t<decltype(std::declval<Deref>()(std::declval<Primitive>()))>::type,
      typename Scalar = typename scalar_t<Vector>::type>
    class aabb_tree
    {
    public:
      using vector = Vector;
      static constexpr std::size_t dimension = dimension_t<vector>::value;
      using scalar = Scalar;
      using aabb = interval<scalar, dimension, false, false>;


      //abstract base class defining the common interface of all aabb tree node
      class aabb_node
      {
      public:
        aabb_node()
        {}

        aabb_node(const aabb &b) : bounds_(b)
        {}

        //returns the bounding box of the node
        const aabb &bounds() const
        {
          return bounds_;
        }

        virtual std::unique_ptr<aabb_node> clone() const = 0;

        //this method must be implemented to return true for a leaf node and false for a non_lef node
        virtual bool is_leaf() const = 0;

        //virtual destructor
        virtual ~aabb_node()
        {}

      protected:
        //storage of bounding box associated with aabb_node
        aabb bounds_;
      };

      ///a class representing a leaf node of an aabb tree (non split node)
      class aabb_leaf_node : public aabb_node
      {
      public:

        aabb_leaf_node() = default;

        aabb_leaf_node(const aabb_leaf_node &) = default;

        aabb_leaf_node(aabb_leaf_node &&) noexcept = default;

        aabb_leaf_node &operator=(const aabb_leaf_node &) = default;

        aabb_leaf_node &operator=(aabb_leaf_node &&) noexcept = default;

        //construct a leaf node from
        aabb_leaf_node(const std::vector<Primitive> &primitives, const aabb &b)
          : aabb_node(b), primitives_(primitives)
        {
        }

        const std::vector<Primitive> &primitives() const
        {
          return primitives_;
        }

        std::unique_ptr<aabb_node> clone() const override
        {
          return std::make_unique<aabb_leaf_node>(*this);
        }

        //return always true because its a leaf node
        bool is_leaf() const override
        {
          return true;
        }

      private:
        std::vector<Primitive> primitives_;

      };

      ///a class representing a split node of an aabb tree (non leaf node)
      class aabb_split_node : public aabb_node
      {
        //child pointers
        std::unique_ptr<aabb_node> children_[2];
      public:
        //default constructor
        aabb_split_node() = default;

        aabb_split_node(const aabb_split_node &) = default;

        aabb_split_node(aabb_split_node &&) noexcept = default;

        aabb_split_node &operator=(const aabb_split_node &) = default;

        aabb_split_node &operator=(aabb_split_node &&) noexcept = default;

        aabb_split_node(const aabb &b)
          : aabb_node(b)
        {}

        //construct a split node from given left and right child pointers and given bounding box b of the node
        aabb_split_node(aabb_node *left_child, aabb_node *right_child, const aabb &b)
          : aabb_node(b)
          , children_{std::unique_ptr<aabb_node>(left_child), std::unique_ptr<aabb_node>(right_child)}
        {
        }

        std::unique_ptr<aabb_node> clone() const override
        {
          auto copy = std::make_unique<aabb_split_node>(this->bounds());
          if (left())
            copy->left() = left()->clone();
          if (right())
            copy->right() = right()->clone();
          return copy;
        }


        //returns always false because its a split node
        bool is_leaf() const override
        {
          return false;
        }

        //returns a pointer to the left child node
        std::unique_ptr<aabb_node> &left()
        {
          return children_[0];
        }

        //returns a pointer to the right child node
        std::unique_ptr<aabb_node> &right()
        {
          return children_[1];
        }

        //returns a const pointer to the left child node
        const std::unique_ptr<aabb_node> &left() const
        {
          return children_[0];
        }

        //returns a const pointer to the right child node
        const std::unique_ptr<aabb_node> &right() const
        {
          return children_[1];
        }

      };


      aabb_tree(Deref deref = {},std::size_t max_depth = 20, std::size_t min_size = 2, scalar box_offset_eps = 0.001 )
        : max_depth_(max_depth), min_size_(min_size), root_(nullptr), box_offset_eps_(box_offset_eps), deref_(deref)
      {
      }

      //copy constructor
      aabb_tree(const aabb_tree &other)
        : max_depth_ (other.max_depth_)
        , min_size_( other.min_size_)
        , root_( other.root_ ? other.root_->clone() : nullptr)
        , deref_( other.deref_)
      {

      }

      //move constructor
      aabb_tree(aabb_tree &&) = default;

      //copy assignment operator
      aabb_tree &operator=(const aabb_tree &other)
      {
        if (this == &other)
          return *this;

        max_depth_ = other.max_depth_;
        min_size_ = other.min_size_;
        root_ = other.root_ ? other.root_->clone() : nullptr;
        deref_ = other.deref_;
        return *this;
      }

      //move assign operator
      aabb_tree &operator=(aabb_tree &&) noexcept = default;

      //remove all primitives from tree
      void clear()
      {
        root_ = nullptr;
      }


      //returns a const pointer to the root node of the tree
      const std::unique_ptr<aabb_node> &root() const
      {
        return root_;
      }

      //construct the tree from all prior inserted primitives
      template<typename PrimitiveIterator>
      void build(PrimitiveIterator begin, PrimitiveIterator end)
      {
        //if tree already constructed -> delete tree
        root_ = nullptr;

        //create list of all primitive handles
        std::vector<Primitive> primitive_list(begin, end);
        auto pend = primitive_list.end();

        aabb b = compute_bounds(primitive_list);

        //initial call to the recursive tree construction method over the whole range of primitives
        root_ = std::unique_ptr<aabb_node>(build(std::move(primitive_list), b, 0));

      }


      //return depth of constructed tree
      std::size_t depth() const
      {
        return depth(root_);
      }

      //return number of all nodes
      std::size_t num_nodes() const
      {
        return num_nodes(root_);
      }

      //return number of split nodes
      std::size_t num_split_nodes() const
      {
        return num_split_nodes(root_);
      }

      //returns number of leaf nodes
      std::size_t num_leaf_nodes() const
      {
        return num_leaf_nodes(root_);
      }

      auto primitive(const Primitive& prim) const
      {
        return deref_(prim);
      }

    protected:

      //return depth of subtree  with node as root
      std::size_t depth(const std::unique_ptr<aabb_node> &node) const
      {
        if (node == nullptr)
          return 0;

        if (node->is_leaf())
          return 1;

        auto snode = static_cast<const aabb_split_node *>(node.get());
        return 1 + (std::max)(depth(snode->left()), depth(snode->right()));
      }

      //count all nodes in subtree  with node as root

      std::size_t num_nodes(const std::unique_ptr<aabb_node> &node) const
      {
        if (node == nullptr)
          return 0;

        if (node->is_leaf())
          return 1;

        const aabb_split_node *snode = static_cast<const aabb_split_node *>(node.get());
        return 1 + num_nodes(snode->left()) + num_nodes(snode->right());
      }

      //count all split nodes in subtree with node as root

      std::size_t num_split_nodes(const std::unique_ptr<aabb_node> &node) const
      {
        if (node == nullptr)
          return 0;

        if (node->is_leaf())
          return 0;

        const aabb_split_node *snode = static_cast<const aabb_split_node *>(node.get());
        return 1 + num_split_nodes(snode->left()) + num_split_nodes(snode->right());
      }

      //count  all leaf nodes in subtree with node as root
      int num_leaf_nodes(const std::unique_ptr<aabb_node> &node) const
      {
        if (node == nullptr)
          return 0;

        if (node->is_leaf())
          return 1;

        const aabb_split_node *snode = static_cast<const aabb_split_node *>(node.get());
        return num_leaf_nodes(snode->left()) + num_leaf_nodes(snode->right());
      }


      //helper function to compute an axis aligned bounding box over the range of primitives [begin,end)
      aabb compute_bounds(const std::vector<Primitive> &primitive_list) const
      {
        aabb b;

        for(const auto& p : primitive_list)
          b.insert(deref_(p));
        b.enlarge(1,box_offset_eps_);
        return b;
      }

      //constructs  tree over all primitives in  primitive_list

      aabb_node *build(std::vector<Primitive> &&primitive_list, const aabb &b, std::size_t depth)
      {

        if (depth >= max_depth_ || primitive_list.size() <= min_size_)
          return new aabb_leaf_node(primitive_list, b);

        vector e = b.extents();
        std::size_t axis = e.max_abs_element_index();

        auto mid = primitive_list.begin() + primitive_list.size() / 2;
        std::nth_element(primitive_list.begin(), mid, primitive_list.end(),
                         [&](const Primitive &a, const Primitive &b)
                         { return reference_point(deref_(a))[axis] < reference_point(deref_(b))[axis]; }
        );

        std::vector<Primitive> right_primitive_list(mid, primitive_list.end());
        aabb rbounds = compute_bounds(right_primitive_list);

        primitive_list.erase(mid, primitive_list.end());
        aabb lbounds = compute_bounds(primitive_list);

        return new aabb_split_node(build(std::move(primitive_list), lbounds, depth + 1),
                                   build(std::move(right_primitive_list), rbounds, depth + 1), b);

      }



    private:

      //std::function<aabb(const Primitive&)> compute_bounds;

      //maximum allowed tree depth to stop tree construction
      std::size_t max_depth_;
      //minimal number of primitives to stop tree construction
      std::size_t min_size_;
      //pointer to the root node of the tree
      std::unique_ptr<aabb_node> root_;
      //all bounding boxes are enlarged by this offset to avoid floating-point errors in ray segment itersections routines
      scalar box_offset_eps_;

      Deref deref_;
    };

    template<typename PrimitiveIterator>
    auto make_aabb_tree(PrimitiveIterator begin, PrimitiveIterator end)
    {

      aabb_tree<std::decay_t<decltype(*begin)>> tree;
      tree.build(begin, end);
      return tree;
    }

    template<typename PrimitiveRange>
    auto make_aabb_tree(const PrimitiveRange &primitives)
    {
      return make_aabb_tree(std::begin(primitives), std::end(primitives));
    }





    //search entry used internally for nearest and k nearest primitive queries

    template<typename Primitive,
      typename Deref = detail::default_deref,
      typename Vector = typename vector_t<decltype(std::declval<Deref>()(std::declval<Primitive>()))>::type,
      typename Scalar = typename scalar_t<Vector>::type>
    class knn_searcher
    {
    public:
      using aabb_tree =  aabb_tree<Primitive,Deref, Vector, Scalar>;
      using scalar =  typename aabb_tree::scalar;
      using vector = typename aabb_tree::vector;
      using aabb_node = typename aabb_tree::aabb_node;
      using aabb_split_node = typename aabb_tree::aabb_split_node;
      using aabb_leaf_node = typename aabb_tree::aabb_leaf_node;

      struct search_entry
      {
        //squared distance to node from query point
        scalar sqr_distance;
        //node
        const aabb_node *node;

        //constructor
        search_entry(const aabb_node *node, scalar sqr_distance)
          : sqr_distance(sqr_distance), node(node)
        {}

        //search entry a < b means a.sqr_distance > b. sqr_distance
        bool operator<(const search_entry &e) const
        {
          return sqr_distance > e.sqr_distance;
        }
      };

      //result entry for nearest and k nearest primitive queries
      struct result_entry
      {
        //squared distance from query point to primitive
        scalar sqr_distance;
        //pointer to primitive
        Primitive primitive;

        //default constructor
        result_entry()
          : sqr_distance(std::numeric_limits<scalar>::infinity())
        {}

        //constructor
        result_entry(const scalar &sqr_distance, const Primitive &p)
          : sqr_distance(sqr_distance), primitive(p)
        {}

        scalar distance() const
        {
          return std::sqrt(sqr_distance);
        }

        //result_entry are sorted by their sqr_distance using this less than operator
        bool operator<(const result_entry &e) const
        {
          return sqr_distance < e.sqr_distance;
        }
      };


      template<typename PrimitiveRange>
      knn_searcher(const PrimitiveRange &primitives, Deref deref = {})
        : tree_(deref)
      {
        tree_.build(std::begin(primitives), std::end(primitives));
      }

      std::vector<result_entry> closest_k_primitives(std::size_t k, const vector &q) const
      {
        if (tree_.root() == nullptr)
          return std::vector<result_entry>();
        std::priority_queue<result_entry> kbest;
        std::priority_queue<search_entry> queue;
        queue.push(search_entry(tree_.root().get(), tree_.root()->bounds().sqr_distance(q)));

        while (!queue.empty())
        {
          search_entry entry = queue.top();
          queue.pop();
          scalar kbest_dist = std::numeric_limits<scalar>::infinity();

          if (kbest.size() == k)
            kbest_dist = kbest.top().sqr_distance;

          if (entry.sqr_distance > kbest_dist)
            break;

          if (entry.node->is_leaf())
          {
            const aabb_leaf_node *leaf = static_cast<const aabb_leaf_node *>(entry.node);
            for (const auto &prim : leaf->primitives())
            {
              auto p = closest_point(tree_.primitive(prim), q);
              scalar dist = sqr_distance(tree_.primitive(prim), q);
              if (dist < kbest_dist)
              {
                if (kbest.size() == k)
                  kbest.pop();
                kbest.push(result_entry(dist, prim));
              }
            }
          } else //not a leaf
          {
            const aabb_split_node *split = static_cast<const aabb_split_node *>(entry.node);

            aabb_node *left = split->left().get();
            scalar left_distance = sqr_distance(left->bounds(), q);

            aabb_node *right = split->right().get();
            scalar right_distance = sqr_distance(right->bounds(), q);

            queue.push(search_entry(left, left_distance));
            queue.push(search_entry(right, right_distance));
          }
        }

        std::vector<result_entry> result(kbest.size());
        auto rend = result.end();
        for (auto rit = result.begin(); rit != rend; ++rit)
        {
          *rit = kbest.top();
          kbest.pop();
        }
        return result;
      }

      std::optional<result_entry> closest_primitive(const vector &q) const
      {
        if (tree_.root() == nullptr)
          return std::nullopt;
        result_entry kbest;
        std::priority_queue<search_entry> queue;
        queue.push(search_entry(tree_.root().get(), tree_.root()->bounds().sqr_distance(q)));

        while (!queue.empty())
        {
          search_entry entry = queue.top();
          queue.pop();

          if (entry.sqr_distance > kbest.sqr_distance)
            break;

          if (entry.node->is_leaf())
          {
            const aabb_leaf_node *leaf = static_cast<const aabb_leaf_node *>(entry.node);
            for (const auto &prim : leaf->primitives())
            {
              auto p = closest_point(tree_.primitive(prim), q);
              scalar dist = sqr_distance(tree_.primitive(prim), q);
              if (dist < kbest.sqr_distance)
              {
                kbest = result_entry(dist, prim);
              }
            }
          }
          else //not a leaf
          {
            const aabb_split_node *split = static_cast<const aabb_split_node *>(entry.node);

            aabb_node *left = split->left().get();
            scalar left_distance = sqr_distance(left->bounds(), q);

            aabb_node *right = split->right().get();
            scalar right_distance = sqr_distance(right->bounds(), q);

            queue.push(search_entry(left, left_distance));
            queue.push(search_entry(right, right_distance));
          }
        }

        return kbest;
      }

    private:
      aabb_tree tree_;

    };








    /*  struct intersection_entry
      {
        //squared distance to node from query point
        ray_segment<scalar,3> ray;
        //node
        const aabb_node* node;

        //constructor
        intersection_entry(const aabb_node* node, const ray_segment<scalar,3>& r):ray(r),node(node){}

        //search entry a < b means a.entry > b. sqr_distance
        bool operator<(const intersection_entry& e) const
        {
          return ray.t_min() > e.ray.t_min();
        }
      };*/
    /*
     *


      //return all primitives whose closest point p has a distance to q <= radius
      virtual std::vector<primitive_handle> query_ball(const vec3& q, scalar radius) const
      {
        assert(is_completed());
        std::vector<primitive_handle> result;
        if(root == nullptr)
          return result;

        scalar r2 = radius*radius;
        std::priority_queue<search_entry> queue;
        queue.push(search_entry(root,root->get_bounds().sqr_distance(q)));
        while(!queue.empty())
        {
          search_entry entry = queue.top();
          queue.pop();

          if(entry.sqr_distance > r2)
            break;

          if(entry.node->is_leaf())
          {
            aabb_leaf_node* leaf = (aabb_leaf_node*)entry.node;
            auto pend = leaf->primitives_end();
            for(auto pit  = leaf->primitives_begin(); pit !=  pend; ++pit)
            {
              scalar dist = geometry_processing::sqr_distance(this->get_primitive(*pit),q);
              if( dist <= r2)
                result.push_back(*pit);
            }
          }
          else //not a leaf
          {

            aabb_split_node* split = (aabb_split_node*)entry.node;

            //if (split node is completely inside of ball)
            //{
            //	add all primitives of subtree
            //}
            //else
            //{

            aabb_node *left = split->left();
            scalar left_distance = geometry_processing::sqr_distance(left->get_bounds(),q);

            aabb_node *right = split->right();
            scalar right_distance = geometry_processing::sqr_distance(right->get_bounds(),q);

            queue.push(search_entry(left,left_distance));
            queue.push(search_entry(right,right_distance));
            //}
          }
        }

        return result;
      }

      //returns the closest primitive to the point q
      typename primitive_list<prim>::result_entry closest_primitive(const vec3& q) const
      {
        assert(is_completed());
        if(root == nullptr) {
          typename primitive_list<prim>::result_entry r;
          return r;
        }

        std::priority_queue<search_entry> queue;
        queue.push(search_entry(root,root->get_bounds().sqr_distance(q)));

        typename primitive_list<prim>::result_entry best;

        while(!queue.empty())
        {
          search_entry entry = queue.top();
          queue.pop();

          if(entry.sqr_distance > best.sqr_distance)
            break;

          if(entry.node->is_leaf())
          {
            aabb_leaf_node* leaf = (aabb_leaf_node*)entry.node;
            auto pend = leaf->primitives_end();
            for(auto pit  = leaf->primitives_begin(); pit !=  pend; ++pit)
            {
              vec3 p = geometry_processing::closest_point(this->get_primitive(*pit),q);
              scalar dist = geometry_processing::sqr_distance(this->get_primitive(*pit),q);
              if(dist < best.sqr_distance)
              {
                best.sqr_distance = dist;
                best.primitive = *pit;
              }
            }
          }
          else //not a leaf
          {
            aabb_split_node* split = (aabb_split_node*)entry.node;

            aabb_node *left = split->left();
            scalar left_distance = geometry_processing::sqr_distance(left->get_bounds(),q);

            aabb_node *right = split->right();
            scalar right_distance = geometry_processing::sqr_distance(right->get_bounds(),q);

            queue.push(search_entry(left,left_distance));
            queue.push(search_entry(right,right_distance));
          }
        }
        return  best;


      }

      //returns true if there exists an intersection between r and a primitive
      bool any_intersection(const ray_segment<scalar,3>& r) const
      {
        int c1 = 0;
        int c2 = 0;
        assert(is_completed());

        if(root == nullptr)
          return false;

        std::stack<const aabb_node*> stack;
        const box<scalar>& b = root->get_bounds();

        if( b.any_intersection(r))
          stack.push(root);

        while(!stack.empty())
        {
          const aabb_node* entry = stack.top();
          stack.pop();

          if(entry->is_leaf())
          {
            aabb_leaf_node* leaf = (aabb_leaf_node*)entry;
            auto pend = leaf->primitives_end();
            for(auto pit  = leaf->primitives_begin(); pit !=  pend; ++pit)
            {
              c1++;
              if(this->get_primitive(*pit).any_intersection(r))
              {
                return true;
              }
            }
          }
          else //not a leaf
          {
            aabb_split_node* split = (aabb_split_node*)entry;

            aabb_node *left = split->left();
            c2+=2;
            if(left->get_bounds().is_inside(r.origin()) || left->get_bounds().any_intersection(r))
              stack.push(left);

            aabb_node *right = split->right();
            if(right->get_bounds().is_inside(r.origin())|| right->get_bounds().any_intersection(r))
              stack.push(right);

          }

        }
        return false;
      }

      //returns the closest intersection between r and the primitives
      typename primitive_list<prim>::intersection_info closest_intersection( const ray_segment<scalar,3>& r) const
      {
        assert(is_completed());
        typename primitive_list<prim>::intersection_info best;
        if(root == nullptr)
          return best;

        std::priority_queue<intersection_entry> queue;
        scalar entry,exit;
        std::tie(entry,exit) = root->get_bounds().intersect(r.origin(),r.direction());
        if(entry < exit) {
          scalar tentry = (std::max)(entry, r.t_min());
          scalar texit = (std::min)(exit, r.t_max());
          queue.push(intersection_entry(root,ray_segment<scalar,3>(r.origin(),r.direction(),tentry,texit)));
        }

        while(!queue.empty())
        {
          intersection_entry entry = queue.top();
          queue.pop();

          if(entry.ray.t_min() > best.distance)
            break;

          if(entry.node->is_leaf())
          {
            aabb_leaf_node* leaf = (aabb_leaf_node*)entry.node;
            auto pend = leaf->primitives_end();
            for(auto pit  = leaf->primitives_begin(); pit !=  pend; ++pit)
            {
              scalar t = geometry_processing::closest_intersection(this->get_primitive(*pit),entry.ray);
              if(t < best.distance)
              {
                best.distance = t;
                best.primitive = *pit;
              }
            }
          }
          else //not a leaf
          {
            aabb_split_node* split = (aabb_split_node*)entry.node;

            aabb_node *left = split->left();
            scalar lentry,lexit;
            std::tie(lentry,lexit) = left->get_bounds().intersect(r.origin(),r.direction());
            if(lentry < lexit) {
              scalar tentry = (std::max)(lentry, r.t_min());
              scalar texit = (std::min)(lexit, r.t_max());
              queue.push(intersection_entry(left,ray_segment<scalar,3>(r.origin(),r.direction(),tentry,texit)));
            }

            aabb_node *right = split->right();
            scalar rentry,rexit;
            std::tie(rentry,rexit) = right->get_bounds().intersect(r.origin(),r.direction());
            if(rentry < rexit) {
              scalar tentry = (std::max)(rentry, r.t_min());
              scalar texit = (std::min)(rexit, r.t_max());
              queue.push(intersection_entry(right,ray_segment<scalar,3>(r.origin(),r.direction(),tentry,texit)));
            }
          }

        }
        return best;


      }

      //return all intersections
      std::vector<typename primitive_list<prim>::intersection_info> all_intersections(const ray_segment<scalar,3>& r) const
      {
        assert(is_completed());

        std::vector<typename primitive_list<prim>::intersection_info> all_intersections;

        if(root == nullptr)
          return all_intersections;

        std::stack<const aabb_node*> stack;
        if(root->get_bounds().any_intersection(r))
          stack.push(root);

        while(!stack.empty())
        {
          const aabb_node* entry = stack.top();
          stack.pop();

          if(entry->is_leaf())
          {
            aabb_leaf_node* leaf = (aabb_leaf_node*)entry;
            auto pend = leaf->primitives_end();
            for(auto pit  = leaf->primitives_begin(); pit !=  pend; ++pit)
            {
              auto intersections = this->get_primitive(*pit).all_intersections(r);
              auto iend = intersections.end();
              for(auto iit = intersections.begin(); iit != iend; ++iend)
              {
                all_intersections.push_back(typename primitive_list<prim>::intersection_info(*pit,*iit));
              }
            }

          }
          else //not a leaf
          {
            aabb_split_node* split = (aabb_split_node*)entry;

            aabb_node *left = split->left();
            if( left->get_bounds().any_intersection(r))
              stack.push(left);

            aabb_node *right = split->right();
            if( right->get_bounds().any_intersection(r))
              stack.push(right);

          }

        }
        return all_intersections;
      }
  */
  }
}





