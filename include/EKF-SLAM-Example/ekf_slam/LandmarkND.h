// This file is part of the Robotic Template Library (RTL), a C++
// template library for usage in robotic research and applications
// under the MIT licence:
//
// Copyright 2021 Brno University of Technology
//
// Permission is hereby granted, free of charge, to any person obtaining
// a copy of this software and associated documentation files (the "Software"),
// to deal in the Software without restriction, including without limitation
// the rights to use, copy, modify, merge, publish, distribute, sublicense,
// and/or sell copies of the Software, and to permit persons to whom
// the Software is furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included
// in all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
// EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
// OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
// IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
// DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT
// OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE
// OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
//
// Contact person: Adam Ligocki <adam.ligocki@vutbr.cz>


#ifndef ROBOTICTEMPLATELIBRARY_LANDMARKND_H
#define ROBOTICTEMPLATELIBRARY_LANDMARKND_H

#include "rtl/Core.h"


template<size_t dimension, typename dtype>
class LandmarkND {

public:

    LandmarkND() = delete;
    LandmarkND(rtl::TranslationND<dimension, dtype> tr, int id = -1)
            : translation_{tr} {
        id_ = {id};
    }

    const rtl::TranslationND<dimension, dtype>& translation() const {return translation_;}
    int id() const {return id_;}

private:
    rtl::TranslationND<dimension, dtype> translation_;
    int id_;
};

#endif