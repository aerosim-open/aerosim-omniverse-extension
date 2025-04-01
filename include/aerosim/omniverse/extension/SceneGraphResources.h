#include <nlohmann/json.hpp>
#include <string>

using json = nlohmann::json;

// Resource
struct Resource {
    std::string resource_id;
    std::string resource_type;
};

struct Origin : Resource {
    double latitude, longitude, altitude;

    friend void from_json(const json& j, Origin& o) {
        if (j.contains("latitude")) j.at("latitude").get_to(o.latitude);
        if (j.contains("longitude")) j.at("longitude").get_to(o.longitude);
        if (j.contains("altitude")) j.at("altitude").get_to(o.altitude);
    }
};

struct TimeStamp : Resource {
    double sec, nsec;

    friend void from_json(const json& j, TimeStamp& t) {
        if (j.contains("sec")) j.at("sec").get_to(t.sec);
        if (j.contains("nsec")) j.at("nsec").get_to(t.nsec);
    }
};

struct ViewportConfig : Resource {
    std::string renderer_instance;
    std::string active_camera;

    friend void from_json(const json& j, ViewportConfig& v) {
        if (j.contains("renderer_instance")) j.at("renderer_instance").get_to(v.renderer_instance);
        if (j.contains("active_camera")) j.at("active_camera").get_to(v.active_camera);
    }
};
