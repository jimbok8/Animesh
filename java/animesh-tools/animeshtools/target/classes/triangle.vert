/* Introducing the OpenGL ES 2 Vertex shader
  *
  * The main loop inside the vertex shader gets executed
  * one time for each vertex.
  *
  *      vertex -> *       uniform data -> mat4 projection = ( 1, 0, 0, 0,
  *      (0,1,0)  / \                                          0, 1, 0, 0,
  *              / . \  <- origo (0,0,0)                       0, 0, 1, 0,
  *             /     \                                        0, 0,-1, 1 );
  *  vertex -> *-------* <- vertex
  *  (-1,-1,0)             (1,-1,0) <- attribute data can be used
  *                        (0, 0,1)    for color, position, normals etc.
  *
  * The vertex shader recive input data in form of
  * "uniform" data that are common to all vertex
  * and
  * "attribute" data that are individual to each vertex.
  * One vertex can have several "attribute" data sources enabled.
  *
  * The vertex shader produce output used by the fragment shader.
  * gl_Position are expected to get set to the final vertex position.
  * You can also send additional user defined
  * "varying" data to the fragment shader.
  *
  * Model Translate, Scale and Rotate are done here by matrix-multiplying a
  * projection matrix against each vertex position.
  *
  * The whole vertex shader program are a String containing GLSL ES language
  * http://www.khronos.org/registry/gles/specs/2.0/GLSL_ES_Specification_1.0.17.pdf
  * sent to the GPU driver for compilation.
  */
// For GLSL 1 and 1.1 code i highly recomend to not include a
// GLSL ES language #version line, GLSL ES section 3.4
// Many GPU drivers refuse to compile the shader if #version is different from
// the drivers internal GLSL version.
//
// This demo use GLSL version 1.1 (the implicit version)

#if __VERSION__ >= 130// GLSL 130+ uses in and out
#define attribute in// instead of attribute and varying
#define varying out// used by OpenGL 3 core and later.
#endif

#ifdef GL_ES
precision mediump float;// Precision Qualifiers
precision mediump int;// GLSL ES section 4.5.2
#endif

uniform mat4    uniform_Projection;// Incomming data used by
attribute vec4  attribute_Position;// the vertex shader
attribute vec4  attribute_Color;// uniform and attributes

varying vec4    varying_Color;// Outgoing varying data
// sent to the fragment shader
void main(void)
{
    varying_Color = attribute_Color;
    gl_Position = uniform_Projection * attribute_Position;
}