#pragma once

#include <ostream>

#include "FieldExporter.h"

class MatlabFieldExporter : public FieldExporter {
public:
	/** 
	 * Constructor 
	 */
	MatlabFieldExporter( std::ostream& ostream );


	/**
	 Destructure
	 */
	~MatlabFieldExporter( );

	/**
 	 * Export the field
 	 */
	void exportField( const Field& field ) const override;
};	