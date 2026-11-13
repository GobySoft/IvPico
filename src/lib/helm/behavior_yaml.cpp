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

#include <algorithm>
#include <set>
#include <sstream>

#include <google/protobuf/descriptor.h>
#include <yaml-cpp/yaml.h>

#include "behavior_file.h"

using ivp::interface::BehaviorSet;
using Behavior = ivp::interface::BehaviorSet::Behavior;
using google::protobuf::FieldDescriptor;

namespace
{
std::string indexed(const std::string& base, int i) { return base + "[" + std::to_string(i) + "]"; }

bool is_common(const FieldDescriptor* f) { return f->name() != "type" && f->name() != "param"; }

class YamlReader
{
  public:
    YamlReader(const std::string& filename, std::vector<std::string>& errors,
               ivpico::helm::SourceMap* map)
        : filename_(filename), errors_(errors), map_(map)
    {
    }

    std::optional<BehaviorSet> read(const std::string& text)
    {
        YAML::Node root;
        try
        {
            root = YAML::Load(text);
        }
        catch (const YAML::Exception& e)
        {
            error(e.mark, e.msg);
            return std::nullopt;
        }

        const std::size_t errors_before = errors_.size();
        BehaviorSet set;
        if (root.IsNull())
            return set;
        if (!root.IsMap())
        {
            error(root.Mark(), "a behavior file is a map of initialize, initialize_deferred, "
                               "modes and behaviors");
            return std::nullopt;
        }
        for (const auto& e : entries(root))
        {
            if (e.key == "initialize" || e.key == "initialize_deferred")
                read_initialize(e.value, e.key == "initialize_deferred", set);
            else if (e.key == "modes")
                read_modes(e.value, set);
            else if (e.key == "behaviors")
                read_behaviors(e.value, set);
            // a place for anchors (&name) to merge (<<: *name) elsewhere, as in docker compose
            else if (e.key.rfind("x-", 0) == 0)
                continue;
            else
                error(e.mark, "unknown key \"" + e.key +
                                  "\" (expected initialize, initialize_deferred, modes, "
                                  "behaviors, or x-... for anchors)");
        }
        if (errors_.size() != errors_before)
            return std::nullopt;
        return set;
    }

  private:
    struct Entry
    {
        std::string key;
        YAML::Node value;
        YAML::Mark mark;
    };

    void error(const YAML::Mark& mark, const std::string& msg)
    {
        errors_.push_back(location(mark).str() + ": " + msg);
    }

    ivpico::helm::SourceLocation location(const YAML::Mark& mark) const
    {
        if (mark.is_null())
            return {filename_, 0, 0};
        return {filename_, mark.line + 1, mark.column + 1};
    }

    void note(const std::string& path, const YAML::Mark& mark)
    {
        if (map_)
            (*map_)[path] = location(mark);
    }

    // A map's entries in order, with merge keys ("<<") expanded; keys given in
    // the map itself win over merged ones
    std::vector<Entry> entries(const YAML::Node& map)
    {
        std::set<std::string> given;
        for (const auto& kv : map)
            if (kv.first.Scalar() != "<<" && !given.insert(kv.first.Scalar()).second)
                error(kv.first.Mark(), "\"" + kv.first.Scalar() + "\" is given twice");

        std::vector<Entry> out;
        std::set<std::string> merged;
        for (const auto& kv : map)
        {
            if (kv.first.Scalar() != "<<")
            {
                out.push_back({kv.first.Scalar(), kv.second, kv.first.Mark()});
                continue;
            }
            std::vector<YAML::Node> sources;
            if (kv.second.IsSequence())
                for (const auto& s : kv.second) sources.push_back(s);
            else
                sources.push_back(kv.second);
            for (const auto& source : sources)
            {
                if (!source.IsMap())
                {
                    error(kv.second.Mark(), "a merge (<<) takes a map or a list of maps");
                    continue;
                }
                for (const auto& e : entries(source))
                    if (!given.count(e.key) && merged.insert(e.key).second)
                        out.push_back(e);
            }
        }
        return out;
    }

    // key_mark: where to report a missing value, as yaml-cpp places null nodes poorly
    bool scalar(const YAML::Node& node, const std::string& what, std::string& out,
                const YAML::Mark& key_mark = YAML::Mark::null_mark())
    {
        if (node.IsScalar())
        {
            out = node.Scalar();
            return true;
        }
        if (node.IsNull())
            error(key_mark.is_null() ? node.Mark() : key_mark,
                  what + " needs a value (\"\" for an empty one)");
        else
            error(node.Mark(), what + " takes one value, not a list or map");
        return false;
    }

    // one value, or a list of them
    std::vector<std::pair<std::string, YAML::Mark>>
    scalars(const YAML::Node& node, const std::string& what, const YAML::Mark& key_mark)
    {
        std::vector<std::pair<std::string, YAML::Mark>> out;
        if (node.IsSequence())
        {
            for (const auto& item : node)
            {
                std::string value;
                if (scalar(item, what, value))
                    out.push_back({value, item.Mark()});
            }
        }
        else
        {
            std::string value;
            if (scalar(node, what, value, key_mark))
                out.push_back({value, node.Mark()});
        }
        return out;
    }

    bool expect_map(const YAML::Node& node, const std::string& what)
    {
        if (node.IsMap())
            return true;
        error(node.Mark(), what + " must be a map");
        return false;
    }

    bool expect_list(const YAML::Node& node, const std::string& what)
    {
        if (node.IsSequence())
            return true;
        error(node.Mark(), what + " must be a list");
        return false;
    }

    void read_initialize(const YAML::Node& node, bool deferred, BehaviorSet& set)
    {
        const std::string what = deferred ? "initialize_deferred" : "initialize";
        if (!expect_map(node, what))
            return;
        for (const auto& e : entries(node))
        {
            std::string value;
            if (!scalar(e.value, what + "." + e.key, value, e.mark))
                continue;
            auto& init = *set.add_initialize();
            init.set_var(e.key);
            init.set_value(value);
            init.set_deferred(deferred);
            note(indexed("initialize", set.initialize_size() - 1), e.mark);
        }
    }

    void read_modes(const YAML::Node& node, BehaviorSet& set)
    {
        if (!expect_list(node, "modes"))
            return;
        for (const auto& item : node)
        {
            if (!expect_map(item, "a mode"))
                continue;
            const std::string path = indexed("mode", set.mode_size());
            auto& mode = *set.add_mode();
            note(path, item.Mark());
            bool has_set = false;
            for (const auto& e : entries(item))
            {
                std::string value;
                if (e.key == "set")
                {
                    has_set = scalar(e.value, "set", value, e.mark);
                    mode.set_set(value);
                    note(path, e.value.Mark());
                }
                else if (e.key == "condition")
                {
                    for (const auto& [c, mark] : scalars(e.value, "condition", e.mark))
                    {
                        note(indexed(path + ".condition", mode.condition_size()), mark);
                        mode.add_condition(c);
                    }
                }
                else if (e.key == "else")
                {
                    if (scalar(e.value, "else", value, e.mark))
                        mode.set_else_value(value);
                    note(path + ".else_value", e.value.Mark());
                }
                else
                {
                    error(e.mark, "unknown mode key \"" + e.key +
                                      "\" (expected set, condition "
                                      "or else)");
                }
            }
            if (!has_set)
                error(item.Mark(), "a mode needs set: MODE_VAR = value");
        }
    }

    void read_behaviors(const YAML::Node& node, BehaviorSet& set)
    {
        if (!expect_list(node, "behaviors"))
            return;
        const auto* refl = Behavior::GetReflection();
        for (const auto& item : node)
        {
            if (!expect_map(item, "a behavior"))
                continue;
            const std::string path = indexed("behavior", set.behavior_size());
            auto& bhv = *set.add_behavior();
            note(path, item.Mark());
            bool has_type = false;
            for (const auto& e : entries(item))
            {
                std::string value;
                const auto* field = Behavior::descriptor()->FindFieldByName(e.key);
                if (e.key == "type")
                {
                    has_type = scalar(e.value, "type", value, e.mark);
                    bhv.set_type(value);
                    note(path, e.value.Mark());
                }
                else if (e.key == "params")
                {
                    if (!expect_map(e.value, "params"))
                        continue;
                    for (const auto& p : entries(e.value))
                    {
                        for (const auto& [v, mark] : scalars(p.value, p.key, p.mark))
                        {
                            note(indexed(path + ".param", bhv.param_size()), mark);
                            auto& param = *bhv.add_param();
                            param.set_key(p.key);
                            param.set_value(v);
                        }
                    }
                }
                else if (field && is_common(field) && field->is_repeated())
                {
                    for (const auto& [v, mark] : scalars(e.value, e.key, e.mark))
                    {
                        note(indexed(path + "." + e.key, refl->FieldSize(bhv, field)), mark);
                        refl->AddString(&bhv, field, v);
                    }
                }
                else if (field && is_common(field))
                {
                    if (scalar(e.value, e.key, value, e.mark))
                        refl->SetString(&bhv, field, value);
                    note(path + "." + e.key, e.value.Mark());
                }
                else
                {
                    error(e.mark, "unknown behavior key \"" + e.key +
                                      "\": the behavior's own parameters go under params");
                }
            }
            if (!has_type)
                error(item.Mark(), "a behavior needs a type (BHV_Waypoint, ...)");
        }
    }

    std::string filename_;
    std::vector<std::string>& errors_;
    ivpico::helm::SourceMap* map_;
};

void emit_values(YAML::Emitter& out, const std::vector<std::string>& values)
{
    if (values.size() == 1)
    {
        out << values.front();
        return;
    }
    out << YAML::BeginSeq;
    for (const auto& v : values) out << v;
    out << YAML::EndSeq;
}

void emit_initialize(YAML::Emitter& out, const BehaviorSet& set, bool deferred)
{
    bool any = false;
    for (const auto& init : set.initialize())
    {
        if (init.deferred() != deferred)
            continue;
        if (!any)
            out << YAML::Key << (deferred ? "initialize_deferred" : "initialize") << YAML::Value
                << YAML::BeginMap;
        any = true;
        out << YAML::Key << init.var() << YAML::Value << init.value();
    }
    if (any)
        out << YAML::EndMap;
}
} // namespace

std::optional<BehaviorSet> ivpico::helm::read_yaml(const std::string& text,
                                                   const std::string& filename,
                                                   std::vector<std::string>& errors, SourceMap* map)
{
    return YamlReader(filename, errors, map).read(text);
}

std::string ivpico::helm::yaml_text(const BehaviorSet& set)
{
    YAML::Emitter out;
    out.SetIndent(2);
    out << YAML::BeginMap;
    emit_initialize(out, set, false);
    emit_initialize(out, set, true);

    if (set.mode_size())
    {
        out << YAML::Key << "modes" << YAML::Value << YAML::BeginSeq;
        for (const auto& mode : set.mode())
        {
            out << YAML::BeginMap << YAML::Key << "set" << YAML::Value << mode.set();
            if (mode.condition_size())
            {
                out << YAML::Key << "condition" << YAML::Value;
                emit_values(out, {mode.condition().begin(), mode.condition().end()});
            }
            if (mode.has_else_value())
                out << YAML::Key << "else" << YAML::Value << mode.else_value();
            out << YAML::EndMap;
        }
        out << YAML::EndSeq;
    }

    if (set.behavior_size())
    {
        const auto* refl = Behavior::GetReflection();
        out << YAML::Key << "behaviors" << YAML::Value << YAML::BeginSeq;
        for (const auto& bhv : set.behavior())
        {
            out << YAML::BeginMap << YAML::Key << "type" << YAML::Value << bhv.type();
            for (int f = 0; f < Behavior::descriptor()->field_count(); ++f)
            {
                const auto* field = Behavior::descriptor()->field(f);
                if (!is_common(field))
                    continue;
                std::vector<std::string> values;
                if (field->is_repeated())
                    for (int j = 0; j < refl->FieldSize(bhv, field); ++j)
                        values.push_back(refl->GetRepeatedString(bhv, field, j));
                else if (refl->HasField(bhv, field))
                    values.push_back(refl->GetString(bhv, field));
                if (values.empty())
                    continue;
                out << YAML::Key << field->name() << YAML::Value;
                emit_values(out, values);
            }

            // a parameter given more than once becomes one list, where it first appears
            std::vector<std::pair<std::string, std::vector<std::string>>> params;
            for (const auto& p : bhv.param())
            {
                auto it = std::find_if(params.begin(), params.end(),
                                       [&](const auto& kv) { return kv.first == p.key(); });
                if (it == params.end())
                    params.push_back({p.key(), {p.value()}});
                else
                    it->second.push_back(p.value());
            }
            if (!params.empty())
            {
                out << YAML::Key << "params" << YAML::Value << YAML::BeginMap;
                for (const auto& [key, values] : params)
                {
                    out << YAML::Key << key << YAML::Value;
                    emit_values(out, values);
                }
                out << YAML::EndMap;
            }
            out << YAML::EndMap;
        }
        out << YAML::EndSeq;
    }
    out << YAML::EndMap;

    // a blank line between sections and between behaviors
    std::istringstream in(out.c_str());
    std::string text, line, previous;
    while (std::getline(in, line))
    {
        const bool section = !line.empty() && line.front() != ' ' && line.front() != '-';
        const bool behavior = line.rfind("  - type:", 0) == 0 || line.rfind("- type:", 0) == 0;
        if (!text.empty() && (section || (behavior && previous != "behaviors:")))
            text += "\n";
        text += line + "\n";
        previous = line;
    }
    return text;
}
