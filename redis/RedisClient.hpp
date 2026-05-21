#pragma once

#include <hiredis/hiredis.h>

#include <Eigen/Dense>
#include <glaze/ext/eigen.hpp>
#include <glaze/glaze.hpp>
#include <iostream>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "glaze_helpers.hpp"

namespace RocemoDmCanDriver {
namespace Communication {

struct redisReplyDeleter {
	void operator()(redisReply* r) { freeReplyObject(r); }
};

struct redisContextDeleter {
	void operator()(redisContext* c) { redisFree(c); }
};

class RedisClient {
public:
	RedisClient() = default;
	RedisClient(const RedisClient&) = delete;
	RedisClient& operator=(const RedisClient&) = delete;

	RedisClient(const std::string& key_namespace_prefix);

	void connect(const std::string& hostname = "127.0.0.1",
				 const int port = 6379,
				 const struct timeval& timeout = {1, 500000});

	void ping();
	std::string get(const std::string& key);
	void set(const std::string& key, const std::string& value);
	void del(const std::string& key);
	bool exists(const std::string& key);
	std::vector<std::string> pipeget(const std::vector<std::string>& keys);
	void pipeset(
		const std::vector<std::pair<std::string, std::string>>& keyvals);
	void pipeset(const std::vector<std::string>& keys,
				 const std::vector<std::string>& values);

	template <typename T>
	T get(const std::string& key) {
		std::string value_str = get(key);
		T return_value;
		const auto error = glz::read<glaze_options>(return_value, value_str);
		if (error) {
			throw std::runtime_error(
				"RedisClient: deserialization error for key " + key);
		}
		return return_value;
	}

	template <typename T>
	void set(const std::string& key, const T& value) {
		std::string buffer = glz::write<glaze_options>(value).value();
		set(key, buffer);
	}

	template <typename T>
	void setCustomStruct(
		const std::vector<std::string>& keys, const T& custom_struct,
		const std::map<std::string, std::vector<std::string>>& map_keys = {}) {
		if (keys.size() != glz::reflect<T>::size) {
			throw std::runtime_error(
				"RedisClient: Number of keys does not match number of fields "
				"in custom struct : " +
				std::string(glz::type_name<T>));
		}
		std::vector<std::pair<std::string, std::string>> keyvals;
		auto key_it = keys.begin();
		// iterate fields in same order as keys
		glz::for_each_field(custom_struct, [&](const auto& field) {
			const std::string& key = *key_it;
			using FieldType = std::decay_t<decltype(field)>;
			if constexpr (glz::is_specialization_v<FieldType, std::map>) {
				// If caller provided explicit map_keys for this key, expand
				// suffixes
				auto mk_it = map_keys.find(key);
				if (mk_it != map_keys.end()) {
					const auto& suffixes = mk_it->second;
					for (const auto& suffix : suffixes) {
						// find the mapped value in the std::map using the
						// suffix as map key
						auto found = field.find(suffix);
						if (found == field.end()) {
							throw std::runtime_error(
								"RedisClient: setCustomStruct missing map key "
								"'" +
								suffix + "' for redis key '" + key + "'.");
						}
						const auto& map_v = found->second;
						std::string buffer =
							glz::write<glaze_options>(map_v).value();
						std::string full_key;
						if (!key.empty() && key.back() == '/') {
							full_key = key + suffix;
						} else {
							full_key = key + "/" + suffix;
						}
						keyvals.emplace_back(full_key, buffer);
					}
				} else {
					// No explicit suffixes provided: serialize the whole map as
					// a single JSON value
					std::string buffer =
						glz::write<glaze_options>(field).value();
					keyvals.emplace_back(key, buffer);
				}
			} else {
				// Non-map field: serialize and set at provided key
				std::string buffer = glz::write<glaze_options>(field).value();
				keyvals.emplace_back(key, buffer);
			}
			++key_it;
		});
		if (!keyvals.empty()) {
			pipeset(keyvals);
		}
	}

	template <typename T>
	T getCustomStruct(
		const std::vector<std::string>& keys,
		const std::map<std::string, std::vector<std::string>>& map_keys = {}) {
		if (keys.size() != glz::reflect<T>::size) {
			throw std::runtime_error(
				"RedisClient: Number of keys does not match number of fields "
				"in custom struct : " +
				std::string(glz::type_name<T>));
		}
		T custom_struct;

		// First pass: compute all full keys to read and metadata to map results
		// back
		struct Meta {
			bool is_map = false;
			bool expanded = false;	// true if map expanded into multiple keys
			size_t start_idx = 0;
			size_t count = 0;
			std::vector<std::string> suffixes;	// only for expanded maps
		};

		std::vector<Meta> metas;
		std::vector<std::string> full_keys;

		auto key_it = keys.begin();
		glz::for_each_field(custom_struct, [&](auto& field) {
			const std::string& key = *key_it;
			using FieldType = std::decay_t<decltype(field)>;
			Meta m;
			if constexpr (glz::is_specialization_v<FieldType, std::map>) {
				m.is_map = true;
				auto mk_it = map_keys.find(key);
				if (mk_it != map_keys.end()) {
					// expanded: one full key per suffix
					m.expanded = true;
					m.start_idx = full_keys.size();
					m.suffixes = mk_it->second;
					m.count = m.suffixes.size();
					for (const auto& suffix : m.suffixes) {
						if (!key.empty() && key.back() == '/') {
							full_keys.push_back(key + suffix);
						} else {
							full_keys.push_back(key + "/" + suffix);
						}
					}
				} else {
					// not expanded: a single key for the whole map
					m.expanded = false;
					m.start_idx = full_keys.size();
					m.count = 1;
					full_keys.push_back(key);
				}
			} else {
				// non-map field: single key
				m.is_map = false;
				m.expanded = false;
				m.start_idx = full_keys.size();
				m.count = 1;
				full_keys.push_back(key);
			}
			metas.push_back(std::move(m));
			++key_it;
		});

		if (full_keys.empty()) return custom_struct;

		// Single batch read
		std::vector<std::string> values = pipeget(full_keys);

		// Second pass: deserialize values into the struct fields using metas
		size_t field_idx = 0;
		glz::for_each_field(custom_struct, [&](auto& field) {
			auto& m = metas[field_idx];
			using FieldType = std::decay_t<decltype(field)>;
			if constexpr (glz::is_specialization_v<FieldType, std::map>) {
				if (!m.is_map) {
					throw std::runtime_error(
						"RedisClient: internal error: expected map field");
				}
				if (m.expanded) {
					field.clear();
					for (size_t j = 0; j < m.count; ++j) {
						typename FieldType::mapped_type v;
						auto error = glz::read<glaze_options>(
							v, values[m.start_idx + j]);
						if (error) {
							throw std::runtime_error(
								"RedisClient: deserialization error for map "
								"value for key '" +
								full_keys[m.start_idx + j] + "'");
						}
						field[m.suffixes[j]] = v;
					}
				} else {
					// whole-map value
					auto error =
						glz::read<glaze_options>(field, values[m.start_idx]);
					if (error) {
						throw std::runtime_error(
							"RedisClient: deserialization error for map "
							"field " +
							full_keys[m.start_idx]);
					}
				}
			} else {
				// non-map field: single value
				auto error =
					glz::read<glaze_options>(field, values[m.start_idx]);
				if (error) {
					throw std::runtime_error(
						"RedisClient: deserialization error for field " +
						keys[field_idx]);
				}
			}
			++field_idx;
		});

		return custom_struct;
	}

private:
	std::unique_ptr<redisContext, redisContextDeleter> _context;
	std::string _prefix = "";

	std::unique_ptr<redisReply, redisReplyDeleter> command(const char* format,
														   ...);

	static constexpr glz::opts glaze_options{.format = glz::JSON,
											 .bools_as_numbers = true};
};

}  // namespace Communication
}  // namespace RocemoDMCANDriver
