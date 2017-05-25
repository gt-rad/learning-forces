//
//  ScreenShot.cpp
//  MassSpringLocomotion
//
//  Created by YuWenhao on 4/28/15.
//  Copyright (c) 2015 YuWenhao. All rights reserved.
//

#include "ScreenShot.h"

// Include OpenGL based on OS
#ifdef __APPLE__
#include <OpenGL/gl.h>
#include <OpenGL/glu.h>
#include <OpenGL/glext.h>
#include <GLUT/glut.h>
#else
#ifdef _WIN32
#include <windows.h>
#endif
#include <GL/gl.h>
#include <GL/glu.h>
#include <GL/glext.h>
#include <GL/glut.h>
#endif

#include <png.h>

using namespace std;

// Images

void flip_image (int w, int h, unsigned char *pixels);

void save_png (const char *filename, int width, int height,
               unsigned char *pixels, bool has_alpha = false);

void save_screenshot (const string &filename) {
    int w = 0, h = 0;
    w += glutGet(GLUT_WINDOW_WIDTH);
    h = max(h, glutGet(GLUT_WINDOW_HEIGHT));
    unsigned char *pixels = new unsigned char[w*h*3];
    int x = 0;
    int wsub = glutGet(GLUT_WINDOW_WIDTH),
    hsub = glutGet(GLUT_WINDOW_HEIGHT);
    unsigned char *pixelsub = new unsigned char[wsub*hsub*3];
    glPixelStorei(GL_PACK_ALIGNMENT, 1);
    glReadPixels(0,0, wsub,hsub, GL_RGB, GL_UNSIGNED_BYTE, pixelsub);
    for (int j = 0; j < hsub; j++)
        for (int i = 0; i < wsub; i++)
            for (int c = 0; c < 3; c++)
                pixels[(x+i+w*j)*3+c] = pixelsub[(i+wsub*j)*3+c];
    x += wsub;
    delete[] pixelsub;
    flip_image(w,h, pixels);
    save_png(filename.c_str(), w,h, pixels);
    delete[] pixels;
}

void flip_image (int w, int h, unsigned char *pixels) {
    for (int j = 0; j < h/2; j++)
        for (int i = 0; i < w; i++)
            for (int c = 0; c < 3; c++)
                swap(pixels[(i+w*j)*3+c], pixels[(i+w*(h-1-j))*3+c]);
}

void save_png (const char *filename, int width, int height,
               unsigned char *pixels, bool has_alpha) {
#ifndef _WIN32
    FILE* file = fopen(filename, "wb");
    if (!file) {
        printf("Couldn't open file %s for writing.\n", filename);
        return;
    }
    png_structp png_ptr = png_create_write_struct(PNG_LIBPNG_VER_STRING, NULL,
                                                  NULL, NULL);
    if (!png_ptr) {
        printf("Couldn't create a PNG write structure.\n");
        fclose(file);
        return;
    }
    png_infop info_ptr = png_create_info_struct(png_ptr);
    if (!info_ptr) {
        printf("Couldn't create a PNG info structure.\n");
        png_destroy_write_struct(&png_ptr, NULL);
        fclose(file);
        return;
    }
    if (setjmp(png_jmpbuf(png_ptr))) {
        printf("Had a problem writing %s.\n", filename);
        png_destroy_write_struct(&png_ptr, &info_ptr);
        fclose(file);
        return;
    }
    png_init_io(png_ptr, file);
    png_set_IHDR(png_ptr, info_ptr, width, height, 8,
                 has_alpha ? PNG_COLOR_TYPE_RGBA : PNG_COLOR_TYPE_RGB,
                 PNG_INTERLACE_NONE, PNG_COMPRESSION_TYPE_DEFAULT,
                 PNG_FILTER_TYPE_DEFAULT);
    int channels = has_alpha ? 4 : 3;
    png_bytep* row_pointers = (png_bytep*) new unsigned char*[height];
    for (int y = 0; y < height; y++)
        row_pointers[y] = (png_bytep) &pixels[y*width*channels];
    png_set_rows(png_ptr, info_ptr, row_pointers);
    png_write_png(png_ptr, info_ptr, PNG_TRANSFORM_IDENTITY, NULL);
    delete[] row_pointers;
    png_destroy_write_struct(&png_ptr, &info_ptr);
    fclose(file);
#endif
}

