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

#include "behavior_file.h"

#include <algorithm>
#include <cctype>
#include <fstream>
#include <sstream>

#include <google/protobuf/descriptor.h>

#include "LogicCondition.h"

using ivp::interface::BehaviorSet;
using Behavior = ivp::interface::BehaviorSet::Behavior;
using google::protobuf::FieldDescriptor;

namespace
{
std::string strip(const std::string& s)
{
    const auto begin = s.find_first_not_of(" \t\r");
    if (begin == std::string::npos)
        return "";
    return s.substr(begin, s.find_last_not_of(" \t\r") - begin + 1);
}

std::string lower(std::string s)
{
    for (auto& c : s) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
    return s;
}

bool starts_with_nocase(const std::string& s, const std::string& prefix)
{
    return s.size() >= prefix.size() && lower(s.substr(0, prefix.size())) == prefix;
}

bool has_white(const std::string& s) { return s.find_first_of(" \t") != std::string::npos; }

std::string indexed(const std::string& base, int i) { return base + "[" + std::to_string(i) + "]"; }

// The parameters every behavior takes: Behavior's fields other than type and param
bool is_common(const FieldDescriptor* f) { return f->name() != "type" && f->name() != "param"; }

// MBUtils' parseStringQ: split on sep outside double quotes
std::vector<std::string> split_quoted(const std::string& s, char sep)
{
    std::vector<std::string> parts(1);
    bool quoted = false;
    for (char c : s)
    {
        if (c == '"')
            quoted = !quoted;
        if (c == sep && !quoted)
            parts.emplace_back();
        else
            parts.back() += c;
    }
    return parts;
}

// Values the Populator would read differently than given
void check_value(const std::string& value, const std::string& path,
                 std::vector<std::string>& errors)
{
    if (value.find('\n') != std::string::npos)
        errors.push_back(path + ": a .bhv value can't span lines");
    if (value.find("//") != std::string::npos)
        errors.push_back(path + ": .bhv reads \"//\" as the start of a comment");
    if (!value.empty() && value.back() == '\\')
        errors.push_back(path + ": .bhv joins a line ending in \"\\\" to the next");
    const std::string s = strip(value);
    if (!s.empty() && (s.front() == '{' || s.front() == '}' || s.back() == '{'))
        errors.push_back(path + ": .bhv splits a brace at the start or end of a line");
}

void check_word(const std::string& word, const std::string& what, const std::string& path,
                std::vector<std::string>& errors)
{
    if (strip(word).empty() || has_white(strip(word)) ||
        word.find_first_of("={},") != std::string::npos)
        errors.push_back(path + ": \"" + word + "\" isn't a valid " + what);
}

// The state machine of Populator_BehaviorSet::handleLine
class BhvReader
{
  public:
    BhvReader(BehaviorSet& set, const std::string& filename, ivpico::helm::SourceMap* map)
        : set_(set), filename_(filename), map_(map)
    {
    }

    bool handle(std::string line, int line_num)
    {
        line_num_ = line_num;
        const auto comment = line.find("//");
        if (comment != std::string::npos)
            line.erase(comment);
        line = strip(line);

        if (line.empty())
        {
            if (mode_ == Mode::MiscDefinedIsh)
                mode_ = Mode::Top;
            else if (mode_ == Mode::SetDefinedIsh)
                close_set();
            return true;
        }
        if (line == "{")
        {
            if (mode_ == Mode::Top)
                mode_ = Mode::MiscDefining;
            else if (mode_ == Mode::BhvDeclared)
                mode_ = Mode::BhvDefining;
            else if (mode_ == Mode::SetDeclared)
                mode_ = Mode::SetDefining;
            else
                return false;
            return true;
        }
        if (line == "}")
        {
            if (mode_ == Mode::MiscDefining)
                mode_ = Mode::MiscDefinedIsh;
            else if (mode_ == Mode::BhvDefining)
                mode_ = Mode::Top;
            else if (mode_ == Mode::SetDefining)
                mode_ = Mode::SetDefinedIsh;
            else
                return false;
            return true;
        }

        if (mode_ == Mode::Top || mode_ == Mode::MiscDefinedIsh || mode_ == Mode::SetDefinedIsh)
        {
            if (starts_with_nocase(line, "initialize ") || starts_with_nocase(line, "initialize_ "))
            {
                if (mode_ == Mode::SetDefinedIsh)
                    close_set();
                mode_ = Mode::Top;
                const auto space = line.find(' ');
                const bool deferred = line.substr(0, space) == "initialize_";
                for (const auto& item : split_quoted(line.substr(space + 1), ','))
                {
                    const std::string pair = strip(item);
                    const auto eq = pair.find('=');
                    const std::string left = strip(pair.substr(0, eq));
                    const std::string right =
                        eq == std::string::npos ? "" : strip(pair.substr(eq + 1));
                    if (left.empty() || has_white(left) || right.empty())
                        return false;
                    auto& init = *set_.add_initialize();
                    init.set_var(left);
                    init.set_value(right);
                    init.set_deferred(deferred);
                    note(indexed("initialize", set_.initialize_size() - 1));
                }
                return true;
            }
            if (starts_with_nocase(line, "behavior"))
            {
                if (mode_ == Mode::SetDefinedIsh)
                    close_set();
                const auto eq = line.find('=');
                set_.add_behavior()->set_type(eq == std::string::npos ? ""
                                                                      : strip(line.substr(eq + 1)));
                note(indexed("behavior", set_.behavior_size() - 1));
                mode_ = Mode::BhvDeclared;
                return true;
            }
            if (starts_with_nocase(line, "set "))
            {
                if (mode_ == Mode::SetDefinedIsh)
                    close_set();
                const std::string head = line.substr(4);
                const auto eq = head.find('=');
                const std::string var = strip(head.substr(0, eq));
                const std::string val = eq == std::string::npos ? "" : strip(head.substr(eq + 1));
                if (var.empty() || val.empty())
                    return false;
                pending_mode_ = BehaviorSet::Mode();
                pending_mode_.set_set(var + " = " + val);
                pending_line_ = line_num_;
                mode_ = Mode::SetDeclared;
                return true;
            }
        }

        switch (mode_)
        {
            case Mode::MiscDefining: return true;
            case Mode::SetDefinedIsh:
                if (has_white(line))
                    return false;
                pending_mode_.set_else_value(line);
                pending_else_line_ = line_num_;
                close_set();
                return true;
            case Mode::BhvDefining: add_config(line); return true;
            case Mode::SetDefining:
            {
                LogicCondition condition;
                condition.setAllowDoubleEquals(false);
                if (!condition.setCondition(line))
                    return false;
                pending_mode_.add_condition(line);
                pending_condition_lines_.push_back(line_num_);
                return true;
            }
            default: return false;
        }
    }

  private:
    enum class Mode
    {
        Top,
        MiscDefining,
        MiscDefinedIsh,
        BhvDeclared,
        BhvDefining,
        SetDeclared,
        SetDefining,
        SetDefinedIsh
    };

    void note(const std::string& path, int line = 0)
    {
        if (map_)
            (*map_)[path] = {filename_, line ? line : line_num_, 0};
    }

    void close_set()
    {
        const std::string path = indexed("mode", set_.mode_size());
        *set_.add_mode() = pending_mode_;
        note(path, pending_line_);
        for (std::size_t i = 0; i < pending_condition_lines_.size(); ++i)
            note(indexed(path + ".condition", static_cast<int>(i)), pending_condition_lines_[i]);
        if (pending_mode_.has_else_value())
            note(path + ".else_value", pending_else_line_);
        pending_condition_lines_.clear();
        mode_ = Mode::Top;
    }

    void add_config(const std::string& line)
    {
        auto& bhv = *set_.mutable_behavior(set_.behavior_size() - 1);
        const std::string base = indexed("behavior", set_.behavior_size() - 1);
        const auto eq = line.find('=');
        const std::string key = strip(line.substr(0, eq));
        const std::string value = eq == std::string::npos ? "" : strip(line.substr(eq + 1));

        const auto* f = Behavior::descriptor()->FindFieldByName(lower(key));
        const auto* refl = Behavior::GetReflection();
        if (f && is_common(f) && f->is_repeated())
        {
            refl->AddString(&bhv, f, value);
            note(indexed(base + "." + f->name(), refl->FieldSize(bhv, f) - 1));
        }
        // a common parameter given twice keeps both, as a param
        else if (f && is_common(f) && !refl->HasField(bhv, f))
        {
            refl->SetString(&bhv, f, value);
            note(base + "." + f->name());
        }
        else
        {
            auto& param = *bhv.add_param();
            param.set_key(key);
            param.set_value(value);
            note(indexed(base + ".param", bhv.param_size() - 1));
        }
    }

    BehaviorSet& set_;
    std::string filename_;
    ivpico::helm::SourceMap* map_;
    Mode mode_{Mode::Top};
    int line_num_{0};
    BehaviorSet::Mode pending_mode_;
    int pending_line_{0};
    int pending_else_line_{0};
    std::vector<int> pending_condition_lines_;
};
} // namespace

std::string ivpico::helm::SourceLocation::str() const
{
    std::string s = file;
    if (line > 0)
        s += ":" + std::to_string(line);
    if (line > 0 && column > 0)
        s += ":" + std::to_string(column);
    return s;
}

std::optional<ivpico::helm::SourceLocation> ivpico::helm::locate(const SourceMap& map,
                                                                 std::string path)
{
    while (true)
    {
        auto it = map.find(path);
        if (it != map.end())
            return it->second;
        const auto cut = path.find_last_of(".[");
        if (cut == std::string::npos || cut == 0)
            return std::nullopt;
        path.erase(cut);
    }
}

bool ivpico::helm::is_yaml_file(const std::string& filename)
{
    const auto dot = filename.rfind('.');
    if (dot == std::string::npos)
        return false;
    const std::string ext = lower(filename.substr(dot));
    return ext == ".yml" || ext == ".yaml";
}

std::vector<ivpico::helm::BhvLine> ivpico::helm::bhv_lines(const BehaviorSet& set,
                                                           std::vector<std::string>& errors)
{
    std::vector<BhvLine> lines;
    auto add = [&](const std::string& text, const std::string& path)
    { lines.push_back({text, path}); };

    for (int i = 0; i < set.initialize_size(); ++i)
    {
        const auto& init = set.initialize(i);
        const std::string path = indexed("initialize", i);
        check_word(init.var(), "variable name", path, errors);
        check_value(init.value(), path, errors);
        if (strip(init.value()).empty())
            errors.push_back(path + ": initialize needs a value");
        const auto& value = init.value();
        if (value.find(',') != std::string::npos &&
            !(value.size() > 1 && value.front() == '"' && value.back() == '"'))
            errors.push_back(path + ": .bhv splits an initialize value at a comma unless it's "
                                    "in double quotes");
        add(std::string(init.deferred() ? "initialize_ " : "initialize ") + init.var() + " = " +
                init.value(),
            path);
    }
    if (set.initialize_size())
        add("", "");

    for (int i = 0; i < set.mode_size(); ++i)
    {
        const auto& mode = set.mode(i);
        const std::string path = indexed("mode", i);
        const auto eq = mode.set().find('=');
        if (eq == std::string::npos || strip(mode.set().substr(0, eq)).empty() ||
            strip(mode.set().substr(eq + 1)).empty())
            errors.push_back(path + ": set needs \"MODE_VAR = value\"");
        check_value(mode.set(), path, errors);
        add("set " + mode.set() + " {", path);
        for (int j = 0; j < mode.condition_size(); ++j)
        {
            const std::string cpath = indexed(path + ".condition", j);
            check_value(mode.condition(j), cpath, errors);
            add("  " + mode.condition(j), cpath);
        }
        if (mode.has_else_value())
        {
            if (strip(mode.else_value()).empty() || has_white(strip(mode.else_value())))
                errors.push_back(path + ".else_value: the else value must be one word");
            check_value(mode.else_value(), path + ".else_value", errors);
            add("} " + mode.else_value(), path + ".else_value");
        }
        else
        {
            add("}", path);
        }
        add("", "");
    }

    const auto* refl = Behavior::GetReflection();
    for (int i = 0; i < set.behavior_size(); ++i)
    {
        const auto& bhv = set.behavior(i);
        const std::string path = indexed("behavior", i);
        check_word(bhv.type(), "behavior type", path, errors);
        add("Behavior = " + bhv.type(), path);
        add("{", path);
        for (int f = 0; f < Behavior::descriptor()->field_count(); ++f)
        {
            const auto* field = Behavior::descriptor()->field(f);
            if (!is_common(field))
                continue;
            const std::string fpath = path + "." + field->name();
            if (field->is_repeated())
            {
                for (int j = 0; j < refl->FieldSize(bhv, field); ++j)
                {
                    const std::string value = refl->GetRepeatedString(bhv, field, j);
                    check_value(value, indexed(fpath, j), errors);
                    add("  " + field->name() + " = " + value, indexed(fpath, j));
                }
            }
            else if (refl->HasField(bhv, field))
            {
                const std::string value = refl->GetString(bhv, field);
                check_value(value, fpath, errors);
                add("  " + field->name() + " = " + value, fpath);
            }
        }
        for (int j = 0; j < bhv.param_size(); ++j)
        {
            const auto& param = bhv.param(j);
            const std::string ppath = indexed(path + ".param", j);
            if (strip(param.key()).empty() || param.key().find_first_of("=\n") != std::string::npos)
                errors.push_back(ppath + ": \"" + param.key() + "\" isn't a valid parameter name");
            check_value(param.key(), ppath, errors);
            check_value(param.value(), ppath, errors);
            add("  " + param.key() + " = " + param.value(), ppath);
        }
        add("}", path);
        add("", "");
    }
    return lines;
}

std::string ivpico::helm::bhv_text(const std::vector<BhvLine>& lines)
{
    std::string text;
    for (const auto& line : lines) text += line.text + "\n";
    return text;
}

BehaviorSet ivpico::helm::read_bhv(const std::string& text, const std::string& filename,
                                   std::vector<std::string>& errors, SourceMap* map)
{
    BehaviorSet set;
    BhvReader reader(set, filename, map);

    // lines ending in "\" are joined to the next, as fileBufferSlash does
    std::istringstream in(text);
    std::string raw;
    int line_num = 0;
    while (std::getline(in, raw))
    {
        const int first_line = ++line_num;
        std::string line = strip(raw);
        while (!line.empty() && line.back() == '\\' && std::getline(in, raw))
        {
            ++line_num;
            line.pop_back();
            line += strip(raw);
        }

        // Populator_BehaviorSet::populate's brace separation
        const bool comment = line.rfind("//", 0) == 0 || line.rfind("#", 0) == 0;
        std::string pre, post;
        if (!comment)
        {
            if (line.size() > 1 && (line.front() == '{' || line.front() == '}'))
            {
                pre = line.substr(0, 1);
                line.erase(0, 1);
            }
            if (line.size() > 1 && line.back() == '{')
            {
                post = "{";
                line.pop_back();
            }
        }
        const bool ok = (pre.empty() || reader.handle(pre, first_line)) &&
                        reader.handle(line, first_line) &&
                        (post.empty() || reader.handle(post, first_line));
        if (!ok)
        {
            errors.push_back(SourceLocation{filename, first_line, 0}.str() +
                             ": can't read the line \"" + strip(raw) + "\"");
            break;
        }
    }
    return set;
}

std::string ivpico::helm::yaml_schema()
{
    std::string behavior_props =
        R"json(        "type": { "type": "string", "pattern": "^\\S+$" })json";
    for (int f = 0; f < Behavior::descriptor()->field_count(); ++f)
    {
        const auto* field = Behavior::descriptor()->field(f);
        if (!is_common(field))
            continue;
        behavior_props += ",\n        \"" + field->name() + "\": { \"$ref\": \"#/$defs/" +
                          (field->is_repeated() ? "values" : "value") + "\" }";
    }
    behavior_props += R"json(,
        "params": {
          "description": "The behavior's own parameters, in order; a list for a parameter given more than once",
          "type": "object",
          "additionalProperties": { "$ref": "#/$defs/values" }
        })json";

    return R"json({
  "$schema": "https://json-schema.org/draft/2020-12/schema",
  "$id": "https://github.com/GobySoft/IvPico/blob/main/share/ivpico/bhv.schema.json",
  "title": "IvPico behavior file (.bhv.yml)",
  "description": "MOOS-IvP behaviors, modes and initial variables, as a .bhv file holds them",
  "type": "object",
  "additionalProperties": false,
  "patternProperties": {
    "^x-": { "description": "Ignored: a place for anchors to merge elsewhere" }
  },
  "properties": {
    "initialize": {
      "description": "Variables posted when the helm starts (initialize)",
      "$ref": "#/$defs/variables"
    },
    "initialize_deferred": {
      "description": "Variables posted unless another app has set them (initialize_)",
      "$ref": "#/$defs/variables"
    },
    "modes": {
      "description": "Hierarchical modes (set), in order",
      "type": "array",
      "items": { "$ref": "#/$defs/mode" }
    },
    "behaviors": {
      "type": "array",
      "items": { "$ref": "#/$defs/behavior" }
    }
  },
  "$defs": {
    "value": { "type": ["string", "number", "boolean"] },
    "values": {
      "oneOf": [
        { "$ref": "#/$defs/value" },
        { "type": "array", "items": { "$ref": "#/$defs/value" } }
      ]
    },
    "variables": {
      "type": "object",
      "propertyNames": { "pattern": "^[^\\s=,{}]+$" },
      "additionalProperties": { "$ref": "#/$defs/value" }
    },
    "mode": {
      "type": "object",
      "additionalProperties": false,
      "required": ["set"],
      "properties": {
        "set": { "description": "MODE_VAR = value", "type": "string", "pattern": "\\S\\s*=\\s*\\S" },
        "condition": { "$ref": "#/$defs/values" },
        "else": { "$ref": "#/$defs/value" }
      }
    },
    "behavior": {
      "type": "object",
      "additionalProperties": false,
      "required": ["type"],
      "properties": {
)json" + behavior_props +
           R"json(
      }
    }
  }
}
)json";
}

bool ivpico::helm::needs_combining(const ivp::interface::HelmConfig& cfg)
{
    return cfg.behaviors_size() > 0 ||
           std::any_of(cfg.behavior_file().begin(), cfg.behavior_file().end(), is_yaml_file);
}

ivpico::helm::CombinedBehaviors
ivpico::helm::combine_behaviors(const ivp::interface::HelmConfig& cfg,
                                std::vector<std::string>& errors)
{
    CombinedBehaviors combined;
    auto add_set = [&](const BehaviorSet& set, const std::string& prefix, const SourceMap* map)
    {
        std::vector<std::string> set_errors;
        const auto lines = bhv_lines(set, set_errors);
        auto where = [&](const std::string& path) -> SourceLocation
        {
            if (map)
                if (auto loc = locate(*map, path))
                    return *loc;
            return {prefix + (path.empty() ? "" : "." + path), 0, 0};
        };
        for (const auto& e : set_errors)
        {
            const auto colon = e.find(": ");
            errors.push_back(where(e.substr(0, colon)).str() + e.substr(colon));
        }
        for (const auto& line : lines)
        {
            combined.text += line.text + "\n";
            combined.line_locations.push_back(where(line.path));
        }
    };

    for (const auto& file : cfg.behavior_file())
    {
        std::ifstream in(file);
        if (!in)
        {
            errors.push_back("Could not find File: " + file);
            continue;
        }
        std::stringstream text;
        text << in.rdbuf();

        if (!is_yaml_file(file))
        {
            // lines ending in "\" are joined here, so line numbers stay the file's own
            std::istringstream lines(text.str());
            std::string line;
            int line_num = 0;
            while (std::getline(lines, line))
            {
                const int first_line = ++line_num;
                std::string joined = strip(line);
                while (!joined.empty() && joined.back() == '\\' && std::getline(lines, line))
                {
                    ++line_num;
                    joined.pop_back();
                    joined += strip(line);
                }
                combined.text += joined + "\n";
                combined.line_locations.push_back({file, first_line, 0});
            }
            combined.text += "\n";
            combined.line_locations.push_back({file, 0, 0});
            continue;
        }
#ifdef HAS_YAML
        SourceMap map;
        if (auto set = read_yaml(text.str(), file, errors, &map))
            add_set(*set, file, &map);
#else
        errors.push_back(file + ": this IvPico was built without yaml-cpp (enable_yaml), so it "
                                "can't read YAML behavior files");
#endif
    }
    for (int i = 0; i < cfg.behaviors_size(); ++i)
        add_set(cfg.behaviors(i), "behaviors[" + std::to_string(i) + "]", nullptr);
    return combined;
}

std::string ivpico::helm::relocate_warning(const std::string& warning, const std::string& path,
                                           const CombinedBehaviors& combined)
{
    auto location = [&](unsigned long line) -> std::string
    {
        if (line >= 1 && line <= combined.line_locations.size())
            return combined.line_locations[line - 1].str();
        return "behaviors";
    };

    // Populator_BehaviorSet: a line it can't read, numbered from 1
    const std::string problem = "Problem with line ";
    const std::string in_file = " in the BehaviorSet file: " + path + " Line: ";
    const auto in_file_pos = warning.find(in_file);
    if (warning.rfind(problem, 0) == 0 && in_file_pos != std::string::npos)
    {
        const std::string number = warning.substr(problem.size(), in_file_pos - problem.size());
        if (!number.empty() && std::all_of(number.begin(), number.end(), ::isdigit))
            return location(std::stoul(number)) + ": can't read the line " +
                   warning.substr(in_file_pos + in_file.size());
    }

    // BehaviorSet: a parameter the behavior rejected, numbered from 0
    const std::string prefix = path + ": Line ";
    if (warning.rfind(prefix, 0) == 0)
    {
        const std::string rest = warning.substr(prefix.size());
        const auto colon = rest.find(": ");
        if (colon != std::string::npos && colon > 0 &&
            std::all_of(rest.begin(), rest.begin() + colon, ::isdigit))
            return location(std::stoul(rest.substr(0, colon)) + 1) + ": the behavior rejected \"" +
                   rest.substr(colon + 2) + "\"";
    }

    std::string out = warning;
    for (auto pos = out.find(path); pos != std::string::npos; pos = out.find(path))
        out.replace(pos, path.size(), "behaviors");
    return out;
}
