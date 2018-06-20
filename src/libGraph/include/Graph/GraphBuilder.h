#pragma once

#include <Eigen/Core>
#include "Element/Element.h"
#include "HierarchicalGraph.h"

struct FieldElement {
	Eigen::Vector3f		m_location;
	Eigen::Vector3f		m_normal;
	Eigen::Vector3f		m_tangent;

	FieldElement( Eigen::Vector3f location, 
				  Eigen::Vector3f normal,
				  Eigen::Vector3f tangent ) : m_location{ location }, m_normal{ normal }, m_tangent{ tangent } {};
};


/*
 * GraphBuilder is responsible for building a graph given a set of input element data
 * The input data should comprise a set of Elements each of which supports methods for
 * obtaining location and normal.  The builder will output a graph based on it's own
 * criteria.
 */
template <class EdgeData>
class GraphBuilder {
public:
	/**
	 * Virtual destructur
	 */
	virtual ~GraphBuilder( ) {};

	/**
	 * Build a graph given the input vector of Elements
	 */
	virtual animesh::Graph<FieldElement *, EdgeData> * build_graph_for_elements( const std::vector<Element>& elements ) const = 0;
};

/**
 * Useful method for merging FieldElements
 */
FieldElement * mergeFieldElements ( const FieldElement* fe1, const FieldElement* fe2 ) {
	FieldElement *fe = new FieldElement(
		(fe1->m_location + fe2->m_location) / 2.0,
		(fe1->m_normal + fe2->m_normal) / 2.0,
		Eigen::Vector3f::Zero());

	Eigen::Vector3f tang = (fe1->m_tangent + fe2->m_tangent);
	Eigen::Vector3f error = tang.dot( fe->m_normal ) * fe->m_normal;
	tang = tang - error;
	tang.normalize();
	fe->m_tangent = tang;
	return fe;
}