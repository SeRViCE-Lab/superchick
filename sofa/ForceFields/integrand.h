#include <cmath>

template<typename value_type, typename function_type>
inline value_type integrator(const value_type a,
                             const value_type b,
                             const value_type atol,
                             const value_type reltol,
                             function_type func);

template<typename value_type>
class radial_stress_c2r::radial_stress_c2r(const value_type& Ri,
                                           const value_type& Ro,
                                           const value_type& ri,
                                           const value_type& C1,
                                           const value_type& C2);

template<typename value_type>
class radial_stress_r2c::radial_stress_r2c(const value_type& ri,
                                           const value_type& ro,
                                           const value_type& Ri,
                                           const value_type& C1,
                                           const value_type& C2);

template<typename value_type>
class pressure_r2c::pressure_r2c(const value_type& ri, const value_type& ro,
                                 const value_type& Ri, const value_type& C1,
                                 const value_type& C2)

template<typename value_type>
class pressure_c2r::pressure_c2r(const value_type& Ri, const value_type& Ro,
                                 const value_type& ri, const value_type& C1,
                                 const value_type& C2)
