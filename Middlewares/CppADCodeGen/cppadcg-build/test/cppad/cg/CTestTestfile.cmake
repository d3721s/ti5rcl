# CMake generated Testfile for 
# Source directory: /home/runyu/Documents/GitHub/ti5rcl/Middlewares/CppADCodeGen/test/cppad/cg
# Build directory: /home/runyu/Documents/GitHub/ti5rcl/Middlewares/CppADCodeGen/cppadcg-build/test/cppad/cg
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(array_view "/home/runyu/Documents/GitHub/ti5rcl/Middlewares/CppADCodeGen/cppadcg-build/test/cppad/cg/array_view")
set_tests_properties(array_view PROPERTIES  _BACKTRACE_TRIPLES "/home/runyu/Documents/GitHub/ti5rcl/Middlewares/CppADCodeGen/test/CMakeLists.txt;100;ADD_TEST;/home/runyu/Documents/GitHub/ti5rcl/Middlewares/CppADCodeGen/test/cppad/cg/CMakeLists.txt;20;add_cppadcg_test;/home/runyu/Documents/GitHub/ti5rcl/Middlewares/CppADCodeGen/test/cppad/cg/CMakeLists.txt;0;")
add_test(inputstream "/home/runyu/Documents/GitHub/ti5rcl/Middlewares/CppADCodeGen/cppadcg-build/test/cppad/cg/inputstream")
set_tests_properties(inputstream PROPERTIES  _BACKTRACE_TRIPLES "/home/runyu/Documents/GitHub/ti5rcl/Middlewares/CppADCodeGen/test/CMakeLists.txt;100;ADD_TEST;/home/runyu/Documents/GitHub/ti5rcl/Middlewares/CppADCodeGen/test/cppad/cg/CMakeLists.txt;21;add_cppadcg_test;/home/runyu/Documents/GitHub/ti5rcl/Middlewares/CppADCodeGen/test/cppad/cg/CMakeLists.txt;0;")
add_test(temporary "/home/runyu/Documents/GitHub/ti5rcl/Middlewares/CppADCodeGen/cppadcg-build/test/cppad/cg/temporary")
set_tests_properties(temporary PROPERTIES  _BACKTRACE_TRIPLES "/home/runyu/Documents/GitHub/ti5rcl/Middlewares/CppADCodeGen/test/CMakeLists.txt;100;ADD_TEST;/home/runyu/Documents/GitHub/ti5rcl/Middlewares/CppADCodeGen/test/cppad/cg/CMakeLists.txt;22;add_cppadcg_test;/home/runyu/Documents/GitHub/ti5rcl/Middlewares/CppADCodeGen/test/cppad/cg/CMakeLists.txt;0;")
add_test(mult_sparsity_pattern "/home/runyu/Documents/GitHub/ti5rcl/Middlewares/CppADCodeGen/cppadcg-build/test/cppad/cg/mult_sparsity_pattern")
set_tests_properties(mult_sparsity_pattern PROPERTIES  _BACKTRACE_TRIPLES "/home/runyu/Documents/GitHub/ti5rcl/Middlewares/CppADCodeGen/test/CMakeLists.txt;100;ADD_TEST;/home/runyu/Documents/GitHub/ti5rcl/Middlewares/CppADCodeGen/test/cppad/cg/CMakeLists.txt;23;add_cppadcg_test;/home/runyu/Documents/GitHub/ti5rcl/Middlewares/CppADCodeGen/test/cppad/cg/CMakeLists.txt;0;")
add_test(multi_object_1 "/home/runyu/Documents/GitHub/ti5rcl/Middlewares/CppADCodeGen/cppadcg-build/test/cppad/cg/multi_object_1")
set_tests_properties(multi_object_1 PROPERTIES  _BACKTRACE_TRIPLES "/home/runyu/Documents/GitHub/ti5rcl/Middlewares/CppADCodeGen/test/CMakeLists.txt;100;ADD_TEST;/home/runyu/Documents/GitHub/ti5rcl/Middlewares/CppADCodeGen/test/cppad/cg/CMakeLists.txt;24;add_cppadcg_test;/home/runyu/Documents/GitHub/ti5rcl/Middlewares/CppADCodeGen/test/cppad/cg/CMakeLists.txt;0;")
subdirs("extra")
subdirs("operations")
subdirs("models")
subdirs("model")
subdirs("patterns")
subdirs("evaluator")
subdirs("solve")
subdirs("dae_index_reduction")
subdirs("support")
