// MIT License
// -----------
//
// Copyright (c) 2021 University of Malaga
// Permission is hereby granted, free of charge, to any person
// obtaining a copy of this software and associated documentation
// files (the "Software"), to deal in the Software without
// restriction, including without limitation the rights to use,
// copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the
// Software is furnished to do so, subject to the following
// conditions:
//
// The above copyright notice and this permission notice shall be
// included in all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
// EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
// OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
// NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
// HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
// WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
// FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
// OTHER DEALINGS IN THE SOFTWARE.
//
// Authors: J. Ricardo Sánchez Ibáñez, Carlos J. Pérez del Pulgar
// Affiliation: Department of Systems Engineering and Automation
// Space Robotics Lab (www.uma.es/space-robotics)

#ifndef __MMERROR__
#define __MMERROR__

enum MMError
{
    NO_ERROR = 0,
    POOR_DEM,
    POOR_CONFIG,
    OOB_ROVER_POS,
    OOB_GOAL_POS,
    OBS_ROVER_POS,
    OBS_GOAL_POS,
    PLAN_WO_SAMPLE,
    UNREACH_GOAL,
    UNCERT_GOAL,
    DEGEN_PATH,
    COLLIDING_PROF,
    DEVIATED_PROF,
    FORB_ARM_POS,
    INCOMPLETE_INPUT,
    NON_RESP_ARM,
    COLLIDING_ARM,
    NON_RESP_ROVER,
    EXCESSIVE_DRIFT,
    UNCERT_HEADING,
    GOAL_TOO_CLOSE,    // The goal is too close to the rover
    BAD_DEM_ALLOC,
    UNFEASIBLE_INIT,    // Initial Operation cannot be computed
    IMPROPER_CALL       // A function called in an improper time
};

#endif
