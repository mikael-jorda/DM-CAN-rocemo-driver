#include "RedisClient.hpp"

#include <iostream>
#include <sstream>

using namespace std;

namespace RocemoDmCanDriver {
namespace Communication {

RedisClient::RedisClient(const std::string& key_namespace_prefix)
	: _prefix(key_namespace_prefix) {}

void RedisClient::connect(const std::string& hostname, const int port,
						  const struct timeval& timeout) {
	// Connect to new server
	_context.reset(nullptr);
	redisContext* c = redisConnectWithTimeout(hostname.c_str(), port, timeout);
	std::unique_ptr<redisContext, redisContextDeleter> context(c);

	// Check for errors
	if (!context)
		throw std::runtime_error(
			"RedisClient: Could not allocate redis context.");
	if (context->err)
		throw std::runtime_error(
			"RedisClient: Could not connect to redis server: " +
			std::string(context->errstr));

	// Save context
	_context = std::move(context);
}

std::unique_ptr<redisReply, redisReplyDeleter> RedisClient::command(
	const char* format, ...) {
	va_list ap;
	va_start(ap, format);
	redisReply* reply = (redisReply*)redisvCommand(_context.get(), format, ap);
	va_end(ap);
	return std::unique_ptr<redisReply, redisReplyDeleter>(reply);
}

void RedisClient::ping() {
	auto reply = command("PING");
	std::cout << "RedisClient: PING " << _context->tcp.host << ":"
			  << _context->tcp.port << std::endl;
	if (!reply) throw std::runtime_error("RedisClient: PING failed.");
	std::cout << "Reply: " << reply->str << std::endl;
}

std::string RedisClient::get(const std::string& key) {
	const std::string key_with_prefix = _prefix + key;
	auto reply = command("GET %s", key_with_prefix.c_str());

	if (!reply || reply->type == REDIS_REPLY_ERROR ||
		reply->type == REDIS_REPLY_NIL)
		throw std::runtime_error("RedisClient: GET '" + key_with_prefix +
								 "' failed.");
	if (reply->type != REDIS_REPLY_STRING)
		throw std::runtime_error("RedisClient: GET '" + key_with_prefix +
								 "' returned non-string value.");

	return reply->str;
}

void RedisClient::set(const std::string& key, const std::string& value) {
	const std::string key_with_prefix = _prefix + key;
	auto reply = command("SET %s %s", key_with_prefix.c_str(), value.c_str());

	if (!reply || reply->type == REDIS_REPLY_ERROR)
		throw std::runtime_error("RedisClient: SET '" + key_with_prefix +
								 "' '" + value + "' failed.");
}

void RedisClient::del(const std::string& key) {
	const std::string key_with_prefix = _prefix + key;
	auto reply = command("DEL %s", key_with_prefix.c_str());

	if (!reply || reply->type == REDIS_REPLY_ERROR)
		throw std::runtime_error("RedisClient: DEL '" + key_with_prefix +
								 "' failed.");
}

bool RedisClient::exists(const std::string& key) {
	const std::string key_with_prefix = _prefix + key;
	auto reply = command("EXISTS %s", key_with_prefix.c_str());

	if (!reply || reply->type == REDIS_REPLY_ERROR ||
		reply->type == REDIS_REPLY_NIL)
		throw std::runtime_error("RedisClient: EXISTS '" + key_with_prefix +
								 "' failed.");
	if (reply->type != REDIS_REPLY_INTEGER)
		throw std::runtime_error("RedisClient: EXISTS '" + key_with_prefix +
								 "' returned non-integer value.");

	return (reply->integer == 1);
}

std::vector<std::string> RedisClient::pipeget(
	const std::vector<std::string>& keys) {
	for (const auto& key : keys) {
		const std::string key_with_prefix = _prefix + key;
		redisAppendCommand(_context.get(), "GET %s", key_with_prefix.c_str());
	}

	std::vector<std::string> values;
	for (const auto& key : keys) {
		redisReply* r;
		if (redisGetReply(_context.get(), (void**)&r) == REDIS_ERR)
			throw std::runtime_error(
				"RedisClient: Pipeline GET command failed for key: " + _prefix +
				key + ".");

		std::unique_ptr<redisReply, redisReplyDeleter> reply(r);
		if (reply->type != REDIS_REPLY_STRING)
			throw std::runtime_error(
				"RedisClient: Pipeline GET command returned non-string value "
				"for key: " +
				_prefix + key + ".");

		values.push_back(reply->str);
	}

	return values;
}

void RedisClient::pipeset(
	const std::vector<std::pair<std::string, std::string>>& keyvals) {
	for (const auto& keyval : keyvals) {
		const std::string key_with_prefix = _prefix + keyval.first;
		redisAppendCommand(_context.get(), "SET %s %s", key_with_prefix.c_str(),
						   keyval.second.c_str());
	}

	for (const auto& keyval : keyvals) {
		redisReply* r;
		if (redisGetReply(_context.get(), (void**)&r) == REDIS_ERR)
			throw std::runtime_error(
				"RedisClient: Pipeline SET command failed for key: " + _prefix +
				keyval.first + ".");

		std::unique_ptr<redisReply, redisReplyDeleter> reply(r);
		if (reply->type == REDIS_REPLY_ERROR)
			throw std::runtime_error(
				"RedisClient: Pipeline SET command failed for key: " + _prefix +
				keyval.first + ".");
	}
}

void RedisClient::pipeset(const std::vector<std::string>& keys,
						  const std::vector<std::string>& values) {
	if (keys.size() != values.size())
		throw std::runtime_error(
			"RedisClient: Number of keys does not match number of values.");
	for (size_t i = 0; i < keys.size(); ++i) {
		const std::string key_with_prefix = _prefix + keys[i];
		redisAppendCommand(_context.get(), "SET %s %s", key_with_prefix.c_str(),
						   values[i].c_str());
	}

	for (size_t i = 0; i < keys.size(); ++i) {
		redisReply* r;
		if (redisGetReply(_context.get(), (void**)&r) == REDIS_ERR)
			throw std::runtime_error(
				"RedisClient: Pipeline SET command failed for key: " + _prefix +
				keys[i] + ".");

		std::unique_ptr<redisReply, redisReplyDeleter> reply(r);
		if (reply->type == REDIS_REPLY_ERROR)
			throw std::runtime_error(
				"RedisClient: Pipeline SET command failed for key: " + _prefix +
				keys[i] + ".");
	}
}

}  // namespace Communication
}  // namespace RocemoFR3
