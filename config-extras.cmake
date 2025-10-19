# ament_export_dependencies fails to propagate Boost components
# so we need to explicitly find Boost here
find_package(Boost REQUIRED COMPONENTS system)
