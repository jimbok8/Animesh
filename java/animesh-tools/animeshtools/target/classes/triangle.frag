/* Introducing the OpenGL ES 2 Fragment shader
  *
  * The main loop of the fragment shader gets executed for each visible
  * pixel fragment on the render buffer.
  *
  *       vertex-> *
  *      (0,1,-1) /f\
  *              /ffF\ <- This fragment F gl_FragCoord get interpolated
  *             /fffff\                   to (0.25,0.25,-1) based on the
  *   vertex-> *fffffff* <-vertex         three vertex gl_Position.
  *  (-1,-1,-1)           (1,-1,-1)
  *
  *
  * All incomming "varying" and gl_FragCoord data to the fragment shader
  * gets interpolated based on the vertex positions.
  *
  * The fragment shader produce and store the final color data output into
  * gl_FragColor.
  *
  * Is up to you to set the final colors and calculate lightning here based on
  * supplied position, color and normal data.
  *
  * The whole fragment shader program are a String containing GLSL ES language
  * http://www.khronos.org/registry/gles/specs/2.0/GLSL_ES_Specification_1.0.17.pdf
  * sent to the GPU driver for compilation.
  */
#if __VERSION__ >= 130
#define varying in
out vec4 mgl_FragColor;
#define texture2D texture
#define gl_FragColor mgl_FragColor
#endif

#ifdef GL_ES
precision mediump float;
precision mediump int;
#endif

varying   vec4    varying_Color;//incomming varying data to the
//frament shader
//sent from the vertex shader
void main (void)
{
    gl_FragColor = varying_Color;
}
