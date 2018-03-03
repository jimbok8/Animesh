#pragma once

#include <ostream>

#include "FieldExporter.h"

class MatlabFieldExporter : public FieldExporter {
public:
	/** 
	 * Constructor 
	 */
	MatlabFieldExporter( std::ostream& out );


	/**
	 Destructure
	 */
	~MatlabFieldExporter( );

	/**
 	 * Export the field
 	 */
	void exportField( const Field& field ) const override;


private:
	void writeHeader( std::ostream& out ) const;
	void writeNormalData( std::ostream& out, const Field& field ) const;
	void writeLocationData( std::ostream& out, const Field& field ) const;
	void writeTangentData( std::ostream& out, const Field& field ) const;
	void writeInt( std::ostream& out, int i ) const;
	void writeShort( std::ostream& out, short s ) const;
	void writeVector3f( std::ostream& out, Eigen::Vector3f& vector ) const;
	int writeVectorsHeader( std::ostream& out, const char * const name,  unsigned int numVectors ) const;

	/** Steram to write to */
	std::ostream& 		m_out;
};	