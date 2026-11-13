// Copyright 2026:
//   GobySoft, LLC
//
// This file is part of IvPico.
//
// IvPico is free software: you can redistribute it and/or modify it under the
// terms of the GNU General Public License as published by the Free Software
// Foundation, either version 2 of the License, or (at your option) any later
// version.
//
// IvPico is distributed in the hope that it will be useful, but WITHOUT ANY
// WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
// FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with IvPico. If not, see <https://www.gnu.org/licenses/>.

// Checks that IvPico can use MOOS-IvP's helm libraries (libivp-helm-dev):
//
//  1. The static behavior factory creates behaviors from lib_behaviors-marine
//     and lib_behaviors-colregs, which loads every helm library.
//  2. The IvP solver finds the peak of a one-function problem over course.
//
// Prints one line per problem and exits non-zero if there were any.

#include <cmath>
#include <iostream>
#include <string>

#include "BFactoryStatic.h"
#include "IvPBehavior.h"
#include "IvPDomain.h"
#include "IvPProblem.h"
#include "ZAIC_PEAK.h"

int main()
{
    int errors = 0;

    IvPDomain domain;
    domain.addDomain("course", 0, 359, 360);
    domain.addDomain("speed", 0, 4, 41);

    BFactoryStatic factory;
    factory.setDomain(domain);
    for (const std::string type : {"BHV_Waypoint", "BHV_AvdColregsV22"})
    {
        IvPBehavior* behavior = factory.newBehavior(type);
        if (!behavior)
        {
            std::cerr << "failed: no " << type << " from BFactoryStatic\n";
            ++errors;
        }
        delete behavior;
    }

    IvPDomain course;
    course.addDomain("course", 0, 359, 360);
    ZAIC_PEAK zaic(course, "course");
    zaic.setSummit(90);
    zaic.setBaseWidth(90);
    zaic.setValueWrap(true);

    IvPProblem problem;
    problem.setDomain(course);
    problem.addOF(zaic.extractIvPFunction());
    problem.solve();
    const double result = problem.getResult("course");
    if (std::fabs(result - 90) > 1)
    {
        std::cerr << "failed: solved course " << result << ", not 90\n";
        ++errors;
    }

    return errors == 0 ? 0 : 1;
}
