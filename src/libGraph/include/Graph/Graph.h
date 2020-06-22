#pragma once

#include <map>
#include <vector>
#include <set>
#include <iostream>
#include <list>
#include <tuple>
#include <unordered_set>
#include <spdlog/spdlog.h>

#include "Path.h"

namespace animesh {

/**
* A Graph representation that can handle hierarchical graphs.
* The Graph is constructed over a set of Nodes and Edges
*/
    template<class NodeData, class EdgeData>
    class Graph {
    public:
        /**
         * A node in the graph.
         * Nodes are containers for the data they represent
         */
        class GraphNode {
        public:
            /**
            * Construct a GraphNode from data
            */
            explicit GraphNode(const NodeData &data) : m_data{data} {}

            /**
            * Return the data from the node
            */
            inline const NodeData &data() const { return m_data; }

            /**
            * Set the data in the node
            */
            inline void set_data(const NodeData & data) { m_data = data;}

        private:
            /** The data held in this node */
            NodeData m_data;
        };
        /* ********************************************************************************
        **                                                                            **
        ** Edge                                                                       **
        **                                                                            **
        ********************************************************************************/

        /**
        * An edge in the graph
        */
        class Edge {
        public:
            Edge(const std::shared_ptr<GraphNode> from_node,
                 const std::shared_ptr<GraphNode> to_node,
                 const EdgeData &edge_data) : m_from_node{from_node},
                m_to_node{to_node},
                m_edge_data{edge_data} { } ;

            std::shared_ptr<GraphNode> from() const { return m_from_node; }

            std::shared_ptr<GraphNode> to() const { return m_to_node; }

            const EdgeData &data() const { return m_edge_data; }

            void set_data(const EdgeData &edge_data) {
                m_edge_data = edge_data;
            }

        private:
            std::shared_ptr<GraphNode> m_from_node;
            std::shared_ptr<GraphNode> m_to_node;
            EdgeData m_edge_data;
        };

        /*  ********************************************************************************
            **                                                                            **
            ** Graph public methods                                                       **
            **                                                                            **
            ********************************************************************************/

        /**
         * Make a graph.
         */
        explicit Graph(bool is_directed = false) : m_num_edges{0} {
            m_is_directed = is_directed;
        }

        /**
         * Make a node for the given data.
         * @param node_data
         * @return
         */
        static std::shared_ptr<GraphNode> make_node( const NodeData& node_data) {
            return std::make_shared<GraphNode>(node_data);
        }

        /**
        * Add a node for the given data. Convenience method.
        * @param data The data to be added.
        */
        std::shared_ptr<GraphNode> add_node(const NodeData &data) {
            auto n = make_node(data);
            add_node(n);
            return n;
        }

        /**
        * Add a node for the given data.
        * @param data The data to be added.
        */
        void add_node(const std::shared_ptr<GraphNode>& node) {
            using namespace std;
            assert(node != nullptr);

            // Check that node doesn't exist
            if (m_adjacency.count(node) != 0) {
                spdlog::error("Ignoring attempt to add node to Graph, it's already there");
                return;
            }
            m_adjacency.emplace(node, std::list<std::pair<shared_ptr<GraphNode>, EdgeData>>{});
        }

        /**
         * Remove the given node (and any incident edges)
         */
        void remove_node(const std::shared_ptr<GraphNode>& node) {
            const auto neighbours = m_adjacency.at(node);
            m_num_edges -= neighbours.size();
            m_adjacency.erase(node);
        }

        /**
         * Add an edge to the graph connecting two existing nodes.
         */
        void add_edge(const std::shared_ptr<GraphNode>& from_node,
                      const std::shared_ptr<GraphNode>& to_node,
                      const EdgeData &edge_data) {
            using namespace std;

            if (has_edge(from_node, to_node)) {
                spdlog::warn("Ignoring attempt to add duplicate edge from {:p} to {:p}", fmt::ptr(from_node), fmt::ptr(to_node));
                return;
            }
            m_adjacency.at(from_node).push_back(make_pair(to_node, edge_data));
            if (!m_is_directed) {
                m_adjacency.at(to_node).push_back(make_pair(from_node, edge_data));
            }
            ++m_num_edges;
        }

        /**
        * Remove an edge from the graph. If the graph is directed it will explicitly
        * remove only an edge from from_node to to_node.
        * <p/>
        */
        void remove_edge(const std::shared_ptr<GraphNode>& from_node,
                         const std::shared_ptr<GraphNode>& to_node) {
            if (!has_edge(from_node, to_node)) {
                spdlog::error("Ignoring attempt to remove non-existent edge from {:x} to {:x}", fmt::ptr(from_node), fmt::ptr(to_node));
                return;
            }
            m_adjacency.at(from_node).erase(to_node);
            if (!m_is_directed) {
                m_adjacency.at(to_node).erase(from_node);
            }
            --m_num_edges;
        }

        /**
        * @return the number of nodes in this graph
        */
        size_t num_nodes() const { return m_adjacency.size(); }

        /**
         * @Return a vector of node values in the graph.
         */
        std::vector<NodeData> node_data() const {
            using namespace std;

            vector<NodeData> nodes;
            for (auto it = begin(m_adjacency); it != end(m_adjacency); ++it) {
                nodes.push_back(it->first->data());
            }
            return nodes;
        }


        /**
         *
         */
        std::vector<const std::shared_ptr<GraphNode>> nodes() const {
            using namespace std;

            vector<const shared_ptr<GraphNode>> nodes;
            for (auto it = begin(m_adjacency); it != end(m_adjacency); ++it) {
                nodes.push_back(it->first);
            }
            return nodes;
        }

        /**
         *
         */
        std::vector<Edge> edges() const {
            using namespace std;

            vector<Edge> edges;
            for (auto from_it = begin(m_adjacency); from_it != end(m_adjacency); ++from_it) {
                for (auto to_it = begin(from_it->second); to_it != end(from_it->second); ++to_it) {
                    edges.emplace_back(from_it->first, to_it->first, to_it->second);
                }
            }
            return edges;
        }

        /**
        * @return the number of edges in this graph
        */
        size_t num_edges() const { return m_num_edges; }

        /**
        * @return a vector of the neghbours of a given node
        */
        std::vector<std::shared_ptr<GraphNode>> neighbours(const std::shared_ptr<GraphNode>& node) const {
            using namespace std;
            assert( node != nullptr);

            auto nodes_with_edge_data = m_adjacency.at(node);
            vector<shared_ptr<GraphNode>> neighbours;
            for (const auto &n : nodes_with_edge_data) {
                neighbours.push_back(n.first);
            }
            return neighbours;
        }

        /**
        * @return true if there is an edge from node 1 to node 2. If the graph is undirected,
        * this will return true if there is an edge from node 2 to node 1.
        */
        bool has_edge(const std::shared_ptr<GraphNode> &node_a,
                      const std::shared_ptr<GraphNode>& node_b) const {
            using namespace std;

            auto neighbours = m_adjacency.at(node_a);
            for (const auto &n : neighbours) {
                if (n.first == node_b) {
                    return true;
                }
            }
            return false;
        }

    /**
        * Return a vector of cycles in the graph..
        */
        std::vector<Path<std::shared_ptr<GraphNode>>> cycles() const {
            using namespace std;

            vector<Path<shared_ptr<GraphNode>>> cycles;

            // For each node
            for (auto n : nodes()) {

                Path<shared_ptr<GraphNode>> current_path;
                current_path.push_back(n);

                list<Path<shared_ptr<GraphNode>>> paths;
                paths.push_back(current_path);

                bool done = false;

                // Explore all paths from node to find shortest cycle(s)
                while (!done) {
                    // Grow each path by adding a neighbour not in the path
                    list<Path<shared_ptr<GraphNode>>> new_paths;
                    for (const auto &path : paths) {
                        auto node = path.last();
                        auto edges = m_adjacency.at(node);
                        for (auto edge : edges) {
                            if (!path.contains(edge.first) || (path[0] == edge.first && path.length() > 2)) {
                                new_paths.emplace_back(path, edge.first);
                            }
                        }
                    }
                    paths = new_paths;

                    // Check if we have a cycle yet
                    for (const auto &path : paths) {
                        if (path.is_cycle()) {
                            done = true;
                            // Check whether a variant of this cycle is already known
                            bool cycle_is_known = false;
                            for (const auto &known_cycles : cycles) {
                                if (path.is_equivalent_to(known_cycles)) {
                                    cycle_is_known = true;
                                    break;
                                }
                            }

                            // If not, then add this cycle to my list
                            if (!cycle_is_known) {
                                cycles.push_back(path);
                            }
                        }
                    }
                } // Consider next path
            }// Next node starting point
            vector<Path<shared_ptr<GraphNode>>> cycle_vector;
            cycle_vector.assign(begin(cycles), end(cycles));
            return cycle_vector;
        }

    private:
        bool m_is_directed;
        std::map<
                std::shared_ptr<GraphNode>,
                std::list<
                        std::pair<
                                std::shared_ptr<GraphNode>,
                                EdgeData>>> m_adjacency;
        size_t m_num_edges;
    };
}
