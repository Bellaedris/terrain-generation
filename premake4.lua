pgenerator_files = { "src/*.cpp", "src/*.hpp" }

pgenerator_apps = {
  "terrain_generator"
}

for i, name in ipairs(pgenerator_apps) do
  project(name)
  	language "C++"
  	kind "ConsoleApp"
    location "build"
  	targetdir "bin"
    includedirs "src"
  	files ( gkit_files )
  	files ( pgenerator_files )
  	files { "app/" .. name .. ".cpp" }
end



