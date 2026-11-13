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

#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include "helm/behavior_file.h"
#include "helm/helm.h"

namespace
{
const char* usage = R"(Usage:
  ivpico_bhv convert <file.bhv.yml> [-o <file.bhv>]   YAML to the official .bhv
  ivpico_bhv import <file.bhv> [-o <file.bhv.yml>]    .bhv to YAML (comments aren't kept)
  ivpico_bhv check <file> [--domain name:low:high:points]...
                                                      builds the behaviors and reports problems
  ivpico_bhv schema                                   the JSON Schema of .bhv.yml files
)";

const char* schema_url =
    "https://raw.githubusercontent.com/GobySoft/IvPico/main/share/ivpico/bhv.schema.json";

int fail(const std::string& msg)
{
    std::cerr << msg << "\n";
    return 1;
}

bool read_file(const std::string& file, std::string& text)
{
    std::ifstream in(file);
    if (!in)
        return false;
    std::stringstream s;
    s << in.rdbuf();
    text = s.str();
    return true;
}

int write_output(const std::string& text, const std::string& output)
{
    if (output.empty())
    {
        std::cout << text;
        return 0;
    }
    std::ofstream out(output);
    out << text;
    return out ? 0 : fail("Couldn't write " + output);
}

int report(const std::vector<std::string>& errors)
{
    for (const auto& e : errors) std::cerr << e << "\n";
    return errors.empty() ? 0 : 1;
}

int convert(const std::string& file, const std::string& output)
{
#ifdef HAS_YAML
    ivp::interface::HelmConfig cfg;
    cfg.add_behavior_file(file);
    std::vector<std::string> errors;
    const auto combined = ivpico::helm::combine_behaviors(cfg, errors);
    if (!errors.empty())
        return report(errors);
    return write_output(combined.text, output);
#else
    (void)file, (void)output;
    return fail("ivpico_bhv was built without yaml-cpp (enable_yaml), so it can't read YAML");
#endif
}

int import(const std::string& file, const std::string& output)
{
#ifdef HAS_YAML
    std::string text;
    if (!read_file(file, text))
        return fail("Couldn't read " + file);
    std::vector<std::string> errors;
    const auto set = ivpico::helm::read_bhv(text, file, errors);
    if (!errors.empty())
        return report(errors);
    return write_output(std::string("# yaml-language-server: $schema=") + schema_url + "\n" +
                            ivpico::helm::yaml_text(set),
                        output);
#else
    (void)file, (void)output;
    return fail("ivpico_bhv was built without yaml-cpp (enable_yaml), so it can't write YAML");
#endif
}

int check(const std::string& file, const std::vector<std::string>& domains)
{
    ivp::interface::HelmConfig cfg;
    cfg.add_behavior_file(file);
    cfg.set_ownship("ownship");
    for (const auto& d : domains)
    {
        std::vector<std::string> parts;
        std::stringstream s(d);
        for (std::string part; std::getline(s, part, ':');) parts.push_back(part);
        auto& domain = *cfg.add_domain();
        try
        {
            if (parts.size() != 4)
                throw std::invalid_argument(d);
            domain.set_name(parts[0]);
            domain.set_low(std::stod(parts[1]));
            domain.set_high(std::stod(parts[2]));
            domain.set_points(std::stoul(parts[3]));
        }
        catch (const std::exception&)
        {
            return fail("--domain takes name:low:high:points, e.g. course:0:359:360");
        }
    }
    if (domains.empty())
    {
        for (const auto& [name, low, high, points] :
             std::vector<std::tuple<std::string, double, double, unsigned>>{
                 {"course", 0, 359, 360}, {"speed", 0, 5, 26}, {"depth", 0, 500, 501}})
        {
            auto& domain = *cfg.add_domain();
            domain.set_name(name);
            domain.set_low(low);
            domain.set_high(high);
            domain.set_points(points);
        }
    }

    // MOOS-IvP's libraries narrate their work on stdout and stderr
    std::ostringstream quiet;
    auto* out = std::cout.rdbuf(quiet.rdbuf());
    auto* err = std::cerr.rdbuf(quiet.rdbuf());
    std::vector<std::string> warnings;
    {
        ivpico::helm::Helm helm(cfg);
        warnings = helm.config_warnings();
    }
    std::cout.rdbuf(out);
    std::cerr.rdbuf(err);

    if (!warnings.empty())
        return report(warnings);
    std::cout << file << ": OK\n";
    return 0;
}
} // namespace

int main(int argc, char* argv[])
{
    std::vector<std::string> args(argv + 1, argv + argc);
    if (args.empty() || args[0] == "-h" || args[0] == "--help")
    {
        std::cout << usage;
        return args.empty() ? 1 : 0;
    }

    const std::string command = args[0];
    std::string input, output;
    std::vector<std::string> domains;
    for (std::size_t i = 1; i < args.size(); ++i)
    {
        if ((args[i] == "-o" || args[i] == "--domain") && i + 1 < args.size())
            (args[i] == "-o" ? output : domains.emplace_back()) = args[i + 1], ++i;
        else if (input.empty() && args[i].rfind("-", 0) != 0)
            input = args[i];
        else
            return fail(std::string("Unexpected argument: ") + args[i] + "\n" + usage);
    }

    if (command == "schema" && input.empty())
        return write_output(ivpico::helm::yaml_schema(), output);
    if (input.empty())
        return fail(usage);
    if (command == "convert")
        return convert(input, output);
    if (command == "import")
        return import(input, output);
    if (command == "check")
        return check(input, domains);
    return fail("Unknown command: " + command + "\n" + usage);
}
