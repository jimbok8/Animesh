#pragma once

#include <Field/Field.h>

class FieldExporter {
protected:
	/** 
	 * Pure virtual constructor 
	 */
	FieldExporter( );

	/** 
	 * Pure virtual destructor
	 */
	virtual ~FieldExporter( ) = 0;

public:	
	/**
	 * Export the field
	 */
	virtual void exportField( const Field& field ) const;
};