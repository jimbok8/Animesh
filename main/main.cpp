#include <iostream>

#include "angleBetweenVectors.h"

int main( int argc, char * argv[] ) {
	using namespace Eigen;

	Vector3f v1{ 1.0f, 2.0f, 3.0f };
	Vector3f v2{ 1.0f, 2.0f, 3.0f };

	float a = angleBetweenVectors( v1, v2 );

	std::cout << "Angle is " << a << std::endl;
    return 0;
}
