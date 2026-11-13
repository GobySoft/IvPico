// Copyright 2026:
//   GobySoft, LLC
//
// This file is part of IvPico.
//
// IvPico is free software: you can redistribute it and/or modify it under the
// terms of the GNU Lesser General Public License as published by the Free Software
// Foundation, either version 2.1 of the License, or (at your option) any later
// version.
//
// IvPico is distributed in the hope that it will be useful, but WITHOUT ANY
// WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
// FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// along with IvPico. If not, see <https://www.gnu.org/licenses/>.

#ifndef IVPICO_HELM_BEHAVIOR_FILE_H
#define IVPICO_HELM_BEHAVIOR_FILE_H

#include <map>
#include <optional>
#include <string>
#include <vector>

#include "interface/behaviors.pb.h"
#include "interface/config.pb.h"

namespace ivpico
{
namespace helm
{
struct SourceLocation
{
    std::string file;
    // from 1; 0 if unknown
    int line{0};
    int column{0};

    std::string str() const;
};

// Locations of what a reader read, keyed on the path of each value in the
// BehaviorSet ("behavior[2].endflag[0]", "mode[1].condition[0]", ...)
using SourceMap = std::map<std::string, SourceLocation>;

// The location of a path, or of the nearest enclosing path that has one
std::optional<SourceLocation> locate(const SourceMap& map, std::string path);

// One line of .bhv text, and the path of the value it holds
struct BhvLine
{
    std::string text;
    std::string path;
};

// The official .bhv text for a behavior set. Values that .bhv can't hold (a
// newline, "//", a brace where the Populator splits lines) go in errors as
// "path: message", and the lines are still written.
std::vector<BhvLine> bhv_lines(const ivp::interface::BehaviorSet& set,
                               std::vector<std::string>& errors);
std::string bhv_text(const std::vector<BhvLine>& lines);

// A .bhv file as Populator_BehaviorSet reads it. A line the Populator would
// reject ends the read with an error, as it does.
ivp::interface::BehaviorSet read_bhv(const std::string& text, const std::string& filename,
                                     std::vector<std::string>& errors, SourceMap* map = nullptr);

// Whether a behavior file is YAML (.yml or .yaml)
bool is_yaml_file(const std::string& filename);

#ifdef HAS_YAML
// A .bhv.yml file, checked against the schema; errors are "file:line:column: message"
std::optional<ivp::interface::BehaviorSet> read_yaml(const std::string& text,
                                                     const std::string& filename,
                                                     std::vector<std::string>& errors,
                                                     SourceMap* map = nullptr);
std::string yaml_text(const ivp::interface::BehaviorSet& set);
#endif

// Behaviors from YAML files or given inline can't go to the Populator as they
// are: they, and every .bhv file, are combined into one .bhv text instead
bool needs_combining(const ivp::interface::HelmConfig& cfg);

// Each behavior file in the order given, then the inline sets, with the
// location each line came from. Errors (a missing file, YAML that doesn't
// pass the schema, values .bhv can't hold) mean nothing should be populated.
struct CombinedBehaviors
{
    std::string text;
    std::vector<SourceLocation> line_locations;
};
CombinedBehaviors combine_behaviors(const ivp::interface::HelmConfig& cfg,
                                    std::vector<std::string>& errors);

// A Populator warning about the combined file at path, pointing at where the
// line came from instead
std::string relocate_warning(const std::string& warning, const std::string& path,
                             const CombinedBehaviors& combined);

// The JSON Schema of .bhv.yml files
std::string yaml_schema();

} // namespace helm
} // namespace ivpico

#endif
