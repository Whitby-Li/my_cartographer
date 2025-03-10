//
// Created by whitby on 2025-02-04.
//

#include "my_cartographer/common/lua_parameter_dictionary.h"

#include <algorithm>
#include <cmath>
#include <functional>
#include <memory>

namespace my_cartographer
{
  namespace common
  {

    namespace
    {

      // Replace the string at the top of the stack through a quoted version that Lua
      // can read back.
      void QuoteStringOnStack(lua_State *L)
      {
        CHECK(lua_isstring(L, -1)) << "Top of stack is not a string value.";
        int current_index = lua_gettop(L);

        // S: ... string
        lua_pushglobaltable(L);        // S: ... string globals
        lua_getfield(L, -1, "string"); // S: ... string globals <string module>
        lua_getfield(L, -1,
                     "format");          // S: ... string globals <string module> format
        lua_pushstring(L, "%q");         // S: ... string globals <string module> format "%q"
        lua_pushvalue(L, current_index); // S: ... string globals <string module>
                                         // format "%q" string

        lua_call(L, 2, 1);             // S: ... string globals <string module> quoted
        lua_replace(L, current_index); // S: ... quoted globals <string module>

        lua_pop(L, 2); // S: ... quoted
      }

      // Sets the given 'dictionary' as the value of the "this" key in Lua's registry
      // table.
      void SetDictionaryInRegistry(lua_State *L, LuaParameterDictionary *dictionary)
      {
        lua_pushstring(L, "this");
        lua_pushlightuserdata(L, dictionary);
        lua_settable(L, LUA_REGISTRYINDEX);
      }

      // Gets the 'dictionary' from the "this" key in Lua's registry table.
      LuaParameterDictionary *GetDictionaryFromRegistry(lua_State *L)
      {
        lua_pushstring(L, "this");
        lua_gettable(L, LUA_REGISTRYINDEX);
        void *return_value = lua_isnil(L, -1) ? nullptr : lua_touserdata(L, -1);
        lua_pop(L, 1);
        CHECK(return_value != nullptr);
        return reinterpret_cast<LuaParameterDictionary *>(return_value);
      }

      // CHECK()s if a Lua method returned an error.
      void CheckForLuaErrors(lua_State *L, int status)
      {
        CHECK_EQ(status, 0) << lua_tostring(L, -1);
      }

      // Returns 'a' if 'condition' is true, else 'b'.
      int LuaChoose(lua_State *L)
      {
        CHECK_EQ(lua_gettop(L), 3) << "choose() takes (condition, a, b).";
        CHECK(lua_isboolean(L, 1)) << "condition is not a boolean value.";

        const bool condition = lua_toboolean(L, 1);
        if (condition)
        {
          lua_pushvalue(L, 2);
        }
        else
        {
          lua_pushvalue(L, 3);
        }
        return 1;
      }

      // Pushes a value to the Lua stack.
      void PushValue(lua_State *L, const int key) { lua_pushinteger(L, key); }
      void PushValue(lua_State *L, const std::string &key)
      {
        lua_pushstring(L, key.c_str());
      }

      // Reads the value with the given 'key' from the Lua dictionary and pushes it to
      // the top of the stack.
      template <typename T>
      void GetValueFromLuaTable(lua_State *L, const T &key)
      {
        PushValue(L, key);
        lua_rawget(L, -2);
      }

      // CHECK() that the topmost parameter on the Lua stack is a table.
      void CheckTableIsAtTopOfStack(lua_State *L)
      {
        CHECK(lua_istable(L, -1)) << "Topmost item on Lua stack is not a table!";
      }

      // Returns true if 'key' is in the table at the top of the Lua stack.
      template <typename T>
      bool HasKeyOfType(lua_State *L, const T &key)
      {
        CheckTableIsAtTopOfStack(L);
        PushValue(L, key);
        lua_rawget(L, -2);
        const bool key_not_found = lua_isnil(L, -1);
        lua_pop(L, 1); // Pop the item again.
        return !key_not_found;
      }

      // Iterates over the integer keys of the table at the top of the stack of 'L•
      // and pushes the values one by one. 'pop_value' is expected to pop a value and
      // put them into a C++ container.
      void GetArrayValues(lua_State *L, const std::function<void()> &pop_value)
      {
        int idx = 1;
        while (true)
        {
          GetValueFromLuaTable(L, idx);
          if (lua_isnil(L, -1))
          {
            lua_pop(L, 1);
            break;
          }
          pop_value();
          ++idx;
        }
      }

    } // namespace

    std::unique_ptr<LuaParameterDictionary>
    LuaParameterDictionary::NonReferenceCounted(
        const std::string &code, std::unique_ptr<FileResolver> file_resolver)
    {
      return std::unique_ptr<LuaParameterDictionary>(new LuaParameterDictionary(
          code, ReferenceCount::NO, std::move(file_resolver)));
    }

    LuaParameterDictionary::LuaParameterDictionary(
        const std::string &code, std::unique_ptr<FileResolver> file_resolver)
        : LuaParameterDictionary(code, ReferenceCount::YES,
                                 std::move(file_resolver)) {}

    LuaParameterDictionary::LuaParameterDictionary(
        const std::string &code, ReferenceCount reference_count,
        std::unique_ptr<FileResolver> file_resolver)
        : L_(luaL_newstate()),
          index_into_reference_table_(-1),
          file_resolver_(std::move(file_resolver)),
          reference_count_(reference_count)
    {
      CHECK_NOTNULL(L_);
      SetDictionaryInRegistry(L_, this);

      luaL_openlibs(L_);

      lua_register(L_, "choose", LuaChoose);
      lua_register(L_, "include", LuaInclude);
      lua_register(L_, "read", LuaRead);

      CheckForLuaErrors(L_, luaL_loadstring(L_, code.c_str()));
      CheckForLuaErrors(L_, lua_pcall(L_, 0, 1, 0));
      CheckTableIsAtTopOfStack(L_);
    }

    LuaParameterDictionary::LuaParameterDictionary(
        lua_State *const L, ReferenceCount reference_count,
        std::shared_ptr<FileResolver> file_resolver)
        : L_(lua_newthread(L)),
          file_resolver_(std::move(file_resolver)),
          reference_count_(reference_count)
    {
      CHECK_NOTNULL(L_);

      // Make sure this is never garbage collected.
      CHECK(lua_isthread(L, -1));
      index_into_reference_table_ = luaL_ref(L, LUA_REGISTRYINDEX);

      CHECK(lua_istable(L, -1)) << "Topmost item on Lua stack is not a table!";
      lua_xmove(L, L_, 1); // Moves the table and the coroutine over.
      CheckTableIsAtTopOfStack(L_);
    }

    LuaParameterDictionary::~LuaParameterDictionary()
    {
      if (reference_count_ == ReferenceCount::YES)
      {
        CheckAllKeysWereUsedExactlyOnceAndReset();
      }
      if (index_into_reference_table_ > 0)
      {
        luaL_unref(L_, LUA_REGISTRYINDEX, index_into_reference_table_);
      }
      else
      {
        lua_close(L_);
      }
    }

    std::vector<std::string> LuaParameterDictionary::GetKeys() const
    {
      CheckTableIsAtTopOfStack(L_);
      std::vector<std::string> keys;

      lua_pushnil(L_); // Push the first key
      while (lua_next(L_, -2) != 0)
      {
        lua_pop(L_, 1); // Pop value, keep key.
        if (!lua_isnumber(L_, -1))
        {
          keys.emplace_back(lua_tostring(L_, -1));
        }
      }
      return keys;
    }

    bool LuaParameterDictionary::HasKey(const std::string &key) const
    {
      return HasKeyOfType(L_, key);
    }

    std::string LuaParameterDictionary::GetString(const std::string &key)
    {
      CheckHasKeyAndReference(key);
      GetValueFromLuaTable(L_, key);
      return PopString(Quoted::NO);
    }

    std::string LuaParameterDictionary::PopString(Quoted quoted) const
    {
      CHECK(lua_isstring(L_, -1)) << "Top of stack is not a string value.";
      if (quoted == Quoted::YES)
      {
        QuoteStringOnStack(L_);
      }

      const std::string value = lua_tostring(L_, -1);
      lua_pop(L_, 1);
      return value;
    }

    double LuaParameterDictionary::GetDouble(const std::string &key)
    {
      CheckHasKeyAndReference(key);
      GetValueFromLuaTable(L_, key);
      return PopDouble();
    }

    double LuaParameterDictionary::PopDouble() const
    {
      CHECK(lua_isnumber(L_, -1)) << "Top of stack is not a number value.";
      const double value = lua_tonumber(L_, -1);
      lua_pop(L_, 1);
      return value;
    }

    int LuaParameterDictionary::GetInt(const std::string &key)
    {
      CheckHasKeyAndReference(key);
      GetValueFromLuaTable(L_, key);
      return PopInt();
    }

    int LuaParameterDictionary::PopInt() const
    {
      CHECK(lua_isnumber(L_, -1)) << "Top of stack is not a number value.";
      const int value = lua_tointeger(L_, -1);
      lua_pop(L_, 1);
      return value;
    }

    bool LuaParameterDictionary::GetBool(const std::string &key)
    {
      CheckHasKeyAndReference(key);
      GetValueFromLuaTable(L_, key);
      return PopBool();
    }

    bool LuaParameterDictionary::PopBool() const
    {
      CHECK(lua_isboolean(L_, -1)) << "Top of stack is not a boolean value.";
      const bool value = lua_toboolean(L_, -1);
      lua_pop(L_, 1);
      return value;
    }

    std::unique_ptr<LuaParameterDictionary> LuaParameterDictionary::GetDictionary(
        const std::string &key)
    {
      CheckHasKeyAndReference(key);
      GetValueFromLuaTable(L_, key);
      return PopDictionary(reference_count_);
    }

    std::unique_ptr<LuaParameterDictionary> LuaParameterDictionary::PopDictionary(
        ReferenceCount reference_count) const
    {
      CheckTableIsAtTopOfStack(L_);
      std::unique_ptr<LuaParameterDictionary> value(
          new LuaParameterDictionary(L_, reference_count, file_resolver_));
      // The constructor lua_xmove()s the value, no need to pop it.
      CheckTableIsAtTopOfStack(L_);
      return value;
    }

    std::string LuaParameterDictionary::DoToString(
        const std::string &indent) const
    {
      std::string result = "{";
      bool dictionary_is_empty = true;

      const auto top_of_stack_to_string = [this, indent,
                                           &dictionary_is_empty]() -> std::string
      {
        dictionary_is_empty = false;

        const int value_type = lua_type(L_, -1);
        switch (value_type)
        {
        case LUA_TBOOLEAN:
          return PopBool() ? "true" : "false";
          break;
        case LUA_TSTRING:
          return PopString(Quoted::YES);
          break;
        case LUA_TNUMBER:
        {
          const double value = PopDouble();
          if (std::isinf(value))
          {
            return value < 0 ? "-math.huge" : "math.huge";
          }
          else
          {
            return std::to_string(value);
          }
        }
        break;
        case LUA_TTABLE:
        {
          std::unique_ptr<LuaParameterDictionary> subdict(
              PopDictionary(ReferenceCount::NO));
          return subdict->DoToString(indent + "  ");
        }
        break;
        default:
          LOG(FATAL) << "Unhandled type " << lua_typename(L_, value_type);
        }
      };

      // Integer (array) keys.
      for (int i = 1; i; ++i)
      {
        GetValueFromLuaTable(L_, i);
        if (lua_isnil(L_, -1))
        {
          lua_pop(L_, 1);
          break;
        }
        result.append("\n");
        result.append(indent);
        result.append("  ");
        result.append(top_of_stack_to_string());
        result.append(",");
      }

      // String keys.
      std::vector<std::string> keys = GetKeys();
      if (!keys.empty())
      {
        std::sort(keys.begin(), keys.end());
        for (const std::string &key : keys)
        {
          GetValueFromLuaTable(L_, key);
          result.append("\n");
          result.append(indent);
          result.append("  ");
          result.append(key);
          result.append(" = ");
          result.append(top_of_stack_to_string());
          result.append(",");
        }
      }
      result.append("\n");
      result.append(indent);
      result.append("}");

      if (dictionary_is_empty)
      {
        return "{}";
      }
      return result;
    }

    std::string LuaParameterDictionary::ToString() const { return DoToString(""); }

    std::vector<double> LuaParameterDictionary::GetArrayValuesAsDoubles()
    {
      std::vector<double> values;
      GetArrayValues(L_, [&values, this]
                     { values.push_back(PopDouble()); });
      return values;
    }

    std::vector<std::unique_ptr<LuaParameterDictionary>>
    LuaParameterDictionary::GetArrayValuesAsDictionaries()
    {
      std::vector<std::unique_ptr<LuaParameterDictionary>> values;
      GetArrayValues(L_, [&values, this]
                     { values.push_back(PopDictionary(reference_count_)); });
      return values;
    }

    std::vector<std::string> LuaParameterDictionary::GetArrayValuesAsStrings()
    {
      std::vector<std::string> values;
      GetArrayValues(L_,
                     [&values, this]
                     { values.push_back(PopString(Quoted::NO)); });
      return values;
    }

    void LuaParameterDictionary::CheckHasKey(const std::string &key) const
    {
      CHECK(HasKey(key)) << "Key '" << key << "' not in dictionary:\n"
                         << ToString();
    }

    void LuaParameterDictionary::CheckHasKeyAndReference(const std::string &key)
    {
      CheckHasKey(key);
      reference_counts_[key]++;
    }

    void LuaParameterDictionary::CheckAllKeysWereUsedExactlyOnceAndReset()
    {
      for (const auto &key : GetKeys())
      {
        CHECK_EQ(1, reference_counts_.count(key))
            << "Key '" << key << "' was used the wrong number of times.";
        CHECK_EQ(1, reference_counts_.at(key))
            << "Key '" << key << "' was used the wrong number of times.";
      }
      reference_counts_.clear();
    }

    int LuaParameterDictionary::GetNonNegativeInt(const std::string &key)
    {
      const int temp = GetInt(key); // Will increase reference count.
      CHECK_GE(temp, 0) << temp << " is negative.";
      return temp;
    }

    // Lua function to run a script in the current Lua context. Takes the filename
    // as its only argument.
    int LuaParameterDictionary::LuaInclude(lua_State *L)
    {
      CHECK_EQ(lua_gettop(L), 1);
      CHECK(lua_isstring(L, -1)) << "include takes a filename.";

      LuaParameterDictionary *parameter_dictionary = GetDictionaryFromRegistry(L);
      const std::string basename = lua_tostring(L, -1);
      const std::string filename =
          parameter_dictionary->file_resolver_->GetFullPathOrDie(basename);
      if (std::find(parameter_dictionary->included_files_.begin(),
                    parameter_dictionary->included_files_.end(),
                    filename) != parameter_dictionary->included_files_.end())
      {
        std::string error_msg =
            "Tried to include " + filename +
            " twice. Already included files in order of inclusion: ";
        for (const std::string &filename : parameter_dictionary->included_files_)
        {
          error_msg.append(filename);
          error_msg.append("\n");
        }
        LOG(FATAL) << error_msg;
      }
      parameter_dictionary->included_files_.push_back(filename);
      lua_pop(L, 1);
      CHECK_EQ(lua_gettop(L), 0);

      const std::string content =
          parameter_dictionary->file_resolver_->GetFileContentOrDie(basename);
      CheckForLuaErrors(
          L, luaL_loadbuffer(L, content.c_str(), content.size(), filename.c_str()));
      CheckForLuaErrors(L, lua_pcall(L, 0, LUA_MULTRET, 0));

      return lua_gettop(L);
    }

    // Lua function to read a file into a string.
    int LuaParameterDictionary::LuaRead(lua_State *L)
    {
      CHECK_EQ(lua_gettop(L), 1);
      CHECK(lua_isstring(L, -1)) << "read takes a filename.";

      LuaParameterDictionary *parameter_dictionary = GetDictionaryFromRegistry(L);
      const std::string file_content =
          parameter_dictionary->file_resolver_->GetFileContentOrDie(
              lua_tostring(L, -1));
      lua_pushstring(L, file_content.c_str());
      return 1;
    }

  } // namespace common
} // namespace my_cartographer