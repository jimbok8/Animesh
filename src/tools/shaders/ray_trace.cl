
__constant float PI = 3.14159265359f;

typedef struct Ray {
	float3 origin;
	float3 direction;
} Ray;

typedef struct Camera{
	float3 position;		
	float3 view;			
	float3 up;			
	float2 resolution;	
	float2 fov;		
	float focalDistance;
} Camera;

Ray createCamRay(const int x_coord, const int y_coord, const int width, const int height, const Camera * cam);
bool intersectRayWithFace(const Ray ray, __global float3 * vertices, const int3 face, float * dist );
int intersectRayWithMesh( const Ray ray, const int numVertices, __global float3 * vertices, const int numFaces, __global int3 * faces, float3 * intersection );

Ray createCamRay(const int x_coord, const int y_coord, const int width, const int height, const Camera * cam) {
	/* create a local coordinate frame for the camera */
	float3 rendercamview = normalize(cam->view); 
	float3 rendercamup = normalize(cam->up); 
	float3 horizontalAxis = normalize(cross(rendercamview, rendercamup));
	float3 verticalAxis = normalize(cross(horizontalAxis, rendercamview));

	float3 middle = cam->position + rendercamview;
	float3 horizontal = horizontalAxis * tan(cam->fov.x * 0.5f * (PI / 180)); 
	float3 vertical   =  verticalAxis * tan(cam->fov.y * -0.5f * (PI / 180)); 

	int pixelx = x_coord; 
	int pixely = height - y_coord - 1;

	float sx = (float)pixelx / (width - 1.0f);
	float sy = (float)pixely / (height - 1.0f);
	
	float3 pointOnPlaneOneUnitAwayFromEye = middle + (horizontal * ((2 * sx) - 1)) + (vertical * ((2 * sy) - 1));
	float3 pointOnImagePlane = cam->position + ((pointOnPlaneOneUnitAwayFromEye - cam->position) * cam->focalDistance); /* cam->focalDistance */

	float3 aperturePoint = cam->position;
	float3 apertureToImagePlane = pointOnImagePlane - aperturePoint; apertureToImagePlane = normalize(apertureToImagePlane); 

	/* create camera ray*/
	Ray ray;
	ray.origin = aperturePoint;
	ray.direction = apertureToImagePlane; 

	return ray;
}

bool intersectRayWithFace(const Ray ray,
                          __global float3 * vertices,
                          const int3 face,
                          float * pt ) {
    const float EPSILON = 0.0000001;

    float3 vertex0 = vertices[face.s0];
    float3 vertex1 = vertices[face.s1];
    float3 vertex2 = vertices[face.s2];

    float3 edge1 = vertex1 - vertex0;
    float3 edge2 = vertex2 - vertex0;

    float3 surf_norm = cross(edge1, edge2);
    float  angle = dot(surf_norm, ray.direction);
    if( angle > 0)
    	return false; // This triangle isn't facing me

    float3 h = cross(ray.direction,edge2);
    float a = dot(edge1, h);
    if (a > -EPSILON && a < EPSILON)
        return false;    // This ray is parallel to this triangle.

    float f = 1.0/a;
    float3 s = ray.origin - vertex0;
    float u = f * (dot(s, h));
    if (u < 0.0 || u > 1.0)
        return false;

    float3 q = cross(s,edge1);
    float v = f * dot(ray.direction, q);
    if (v < 0.0 || u + v > 1.0)
        return false;

    // At this stage we can compute t to find out where the intersection point is on the line.
    float t = f * dot(edge2,q);
    // ray intersection
    if (t > EPSILON) {
    	*pt = t;
        return true;
    }
    else // This means that there is a line intersection but not a ray intersection.
        return false;
}

int intersectRayWithMesh( const Ray ray,
                          const int numVertices,
                          __global float3 * vertices,
                          const int numFaces,
                          __global int3 * faces,
                          float3 * intersection ) {


	float dist = INFINITY;
	int bestF = -1;
	for (int f = 0; f < numFaces; f++ ) {
		float idist;
		if ( intersectRayWithFace(ray, vertices, faces[f], &idist)) {
			dist = min( idist, dist);
			bestF = f;
		}
	}
	*intersection = ray.origin + ray.direction * dist;
	return bestF;
}

/**
 * Perform raytracing
 */
__kernel void ray_trace(
    int numVertices,
    __global float3 * vertices,
    int numFaces,
    __global int3 * faces,
    __global float* depth,
    __global unsigned int* vertex,
    int width, int height) {

	Camera camera;
	camera.position      = (float3)(10.0, 10.0, 10.0);
	camera.view          = (float3)(-1.0,  -1.0,  -1.0);
	camera.up            = (float3)(0.0,  1.0,  0.0);
	camera.resolution    = (float2)(640.0, 480.0);
	float fovX = 45.0;
	float aspectRatio    = camera.resolution.y / camera.resolution.x;
	float fovY = (atan(tan(fovX * PI / 360.0) * aspectRatio) * 360.0) / PI;
	camera.fov           = (float2)(fovX, fovY);
	camera.focalDistance = 5.0;

	// Get x,y coord of pixel
	const int work_item_id = get_global_id(0);
	int x = work_item_id % width; /* x-coordinate of the pixel */
	int y = work_item_id / width; /* y-coordinate of the pixel */

	// Create the ray
	Ray ray = createCamRay(x, y, width, height, &camera);

	// Intersect the ray with the mesh
	float3 intersection;
	int intersectedFaceIdx = intersectRayWithMesh(ray, numVertices, vertices, numFaces, faces, &intersection);

	// Compute the depth at this point
	if( intersectedFaceIdx >= 0 ) {
		depth[work_item_id] = intersection.z;

		int3 face = faces[intersectedFaceIdx];
		float d0 = length(intersection - vertices[face.s0]);
		float d1 = length(intersection - vertices[face.s1]);
		float d2 = length(intersection - vertices[face.s2]);

		vertex[work_item_id] = ((d0 > d1) && (d0 > d2)) 
			? face.s0 
			: ((d1 > d0) && (d1 > d2)) 
				? face.s1 
				: face.s2;
	} else {
		depth[work_item_id] = 0.0;
		vertex[work_item_id] = -1;
	}
}

