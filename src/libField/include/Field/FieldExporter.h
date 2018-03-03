#pragma once

#include <Field/Field.h>

class FieldExporter {
public:
	/** 
	 * Pure virtual destructor
	 */
	virtual ~FieldExporter( ) {};

public:	
	/**
	 * Export the field
	 */
	virtual void exportField( const Field& field ) const =0;
};