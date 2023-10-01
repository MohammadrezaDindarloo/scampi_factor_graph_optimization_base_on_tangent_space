// -----------------------------------------------------------------------------
// This file was autogenerated by symforce from template:
//     function/FUNCTION.h.jinja
// Do NOT modify by hand.
// -----------------------------------------------------------------------------

#pragma once

#include <Eigen/Dense>

#include <sym/rot3.h>

namespace sym {

/**
 * This function was autogenerated from a symbolic function. Do not modify by hand.
 *
 * Symbolic function: FK_residual_func_cost3_wrt_fv1
 *
 * Args:
 *     fh1: Scalar
 *     fv1: Scalar
 *     DeltaRot: Rot3
 *     position_vector: Matrix31
 *     rot_init_x: Scalar
 *     rot_init_y: Scalar
 *     rot_init_z: Scalar
 *     rot_init_w: Scalar
 *     lc0: Scalar
 *     lc1: Scalar
 *     lc2: Scalar
 *     lc3: Scalar
 *     epsilon: Scalar
 *
 * Outputs:
 *     res: Matrix41
 */
template <typename Scalar>
Eigen::Matrix<Scalar, 4, 1> FkResidualFuncCost3WrtFv1(
    const Scalar fh1, const Scalar fv1, const sym::Rot3<Scalar>& DeltaRot,
    const Eigen::Matrix<Scalar, 3, 1>& position_vector, const Scalar rot_init_x,
    const Scalar rot_init_y, const Scalar rot_init_z, const Scalar rot_init_w, const Scalar lc0,
    const Scalar lc1, const Scalar lc2, const Scalar lc3, const Scalar epsilon) {
  // Total ops: 288

  // Unused inputs
  (void)lc0;
  (void)lc1;
  (void)lc2;
  (void)lc3;
  (void)epsilon;

  // Input arrays
  const Eigen::Matrix<Scalar, 4, 1>& _DeltaRot = DeltaRot.Data();

  // Intermediate terms (103)
  const Scalar _tmp0 = _DeltaRot[0] * rot_init_z + _DeltaRot[1] * rot_init_w -
                       _DeltaRot[2] * rot_init_x + _DeltaRot[3] * rot_init_y;
  const Scalar _tmp1 = -2 * std::pow(_tmp0, Scalar(2));
  const Scalar _tmp2 = -_DeltaRot[0] * rot_init_y + _DeltaRot[1] * rot_init_x +
                       _DeltaRot[2] * rot_init_w + _DeltaRot[3] * rot_init_z;
  const Scalar _tmp3 = -2 * std::pow(_tmp2, Scalar(2));
  const Scalar _tmp4 = Scalar(0.20999999999999999) * _tmp1 + Scalar(0.20999999999999999) * _tmp3 +
                       Scalar(0.20999999999999999);
  const Scalar _tmp5 = _DeltaRot[0] * rot_init_w - _DeltaRot[1] * rot_init_z +
                       _DeltaRot[2] * rot_init_y + _DeltaRot[3] * rot_init_x;
  const Scalar _tmp6 = 2 * _tmp2 * _tmp5;
  const Scalar _tmp7 = -2 * _DeltaRot[0] * rot_init_x - 2 * _DeltaRot[1] * rot_init_y -
                       2 * _DeltaRot[2] * rot_init_z + 2 * _DeltaRot[3] * rot_init_w;
  const Scalar _tmp8 = _tmp0 * _tmp7;
  const Scalar _tmp9 = -Scalar(0.010999999999999999) * _tmp6 - Scalar(0.010999999999999999) * _tmp8;
  const Scalar _tmp10 = 2 * _tmp0;
  const Scalar _tmp11 = _tmp10 * _tmp5;
  const Scalar _tmp12 = _tmp2 * _tmp7;
  const Scalar _tmp13 = Scalar(0.20999999999999999) * _tmp11 - Scalar(0.20999999999999999) * _tmp12;
  const Scalar _tmp14 = _tmp13 + _tmp9;
  const Scalar _tmp15 = _tmp14 + _tmp4;
  const Scalar _tmp16 = _tmp15 + position_vector(0, 0) + Scalar(-2.71799795);
  const Scalar _tmp17 = 1 - 2 * std::pow(_tmp5, Scalar(2));
  const Scalar _tmp18 = Scalar(0.20999999999999999) * _tmp17 + Scalar(0.20999999999999999) * _tmp3;
  const Scalar _tmp19 = Scalar(0.20999999999999999) * _tmp11 + Scalar(0.20999999999999999) * _tmp12;
  const Scalar _tmp20 = _tmp10 * _tmp2;
  const Scalar _tmp21 = _tmp5 * _tmp7;
  const Scalar _tmp22 =
      -Scalar(0.010999999999999999) * _tmp20 + Scalar(0.010999999999999999) * _tmp21;
  const Scalar _tmp23 = _tmp19 + _tmp22;
  const Scalar _tmp24 = _tmp18 + _tmp23;
  const Scalar _tmp25 = _tmp24 + position_vector(1, 0) + Scalar(-4.7752063900000001);
  const Scalar _tmp26 = std::pow(Scalar(std::pow(_tmp16, Scalar(2)) + std::pow(_tmp25, Scalar(2))),
                                 Scalar(Scalar(-1) / Scalar(2)));
  const Scalar _tmp27 = _tmp16 * _tmp26;
  const Scalar _tmp28 = -_tmp13 + _tmp9;
  const Scalar _tmp29 = _tmp28 + _tmp4;
  const Scalar _tmp30 = _tmp29 + position_vector(0, 0) + Scalar(-2.5202214700000001);
  const Scalar _tmp31 = -_tmp18;
  const Scalar _tmp32 = _tmp23 + _tmp31;
  const Scalar _tmp33 = _tmp32 + position_vector(1, 0) + Scalar(8.3888750099999996);
  const Scalar _tmp34 =
      std::sqrt(Scalar(std::pow(_tmp30, Scalar(2)) + std::pow(_tmp33, Scalar(2))));
  const Scalar _tmp35 = Scalar(1.0) / (_tmp34);
  const Scalar _tmp36 = Scalar(1.0) / (_tmp30);
  const Scalar _tmp37 = _tmp34 * _tmp36;
  const Scalar _tmp38 = _tmp37 * (_tmp29 * _tmp33 * _tmp35 - _tmp30 * _tmp32 * _tmp35);
  const Scalar _tmp39 = _tmp25 * _tmp26;
  const Scalar _tmp40 = -_tmp15 * _tmp39 + _tmp24 * _tmp27 + _tmp27 * _tmp38;
  const Scalar _tmp41 = _tmp33 * _tmp36;
  const Scalar _tmp42 = Scalar(1.0) / (_tmp27 * _tmp41 - _tmp39);
  const Scalar _tmp43 = Scalar(1.0) * _tmp42;
  const Scalar _tmp44 =
      -Scalar(0.010999999999999999) * _tmp1 - Scalar(0.010999999999999999) * _tmp17;
  const Scalar _tmp45 = Scalar(0.20999999999999999) * _tmp6 - Scalar(0.20999999999999999) * _tmp8;
  const Scalar _tmp46 = Scalar(0.20999999999999999) * _tmp20 + Scalar(0.20999999999999999) * _tmp21;
  const Scalar _tmp47 = _tmp44 + _tmp45 - _tmp46;
  const Scalar _tmp48 = _tmp41 * _tmp47;
  const Scalar _tmp49 = _tmp44 + _tmp46;
  const Scalar _tmp50 = _tmp45 + _tmp49;
  const Scalar _tmp51 = -_tmp27 * _tmp48 + _tmp39 * _tmp50;
  const Scalar _tmp52 = Scalar(1.0) * _tmp32;
  const Scalar _tmp53 = Scalar(1.0) * _tmp29;
  const Scalar _tmp54 = (-_tmp15 + _tmp53) / (_tmp24 - _tmp52);
  const Scalar _tmp55 = _tmp27 * _tmp47 - _tmp27 * _tmp50;
  const Scalar _tmp56 = _tmp43 * _tmp51 * _tmp54 - _tmp43 * _tmp55;
  const Scalar _tmp57 = -_tmp19 + _tmp22;
  const Scalar _tmp58 = _tmp18 + _tmp57;
  const Scalar _tmp59 = _tmp58 + position_vector(1, 0) + Scalar(-4.8333311099999996);
  const Scalar _tmp60 = -_tmp4;
  const Scalar _tmp61 = _tmp14 + _tmp60;
  const Scalar _tmp62 = _tmp61 + position_vector(0, 0) + Scalar(1.79662371);
  const Scalar _tmp63 = std::pow(Scalar(std::pow(_tmp59, Scalar(2)) + std::pow(_tmp62, Scalar(2))),
                                 Scalar(Scalar(-1) / Scalar(2)));
  const Scalar _tmp64 = _tmp62 * _tmp63;
  const Scalar _tmp65 = _tmp59 * _tmp63;
  const Scalar _tmp66 = _tmp41 * _tmp64 - _tmp65;
  const Scalar _tmp67 = _tmp42 * _tmp66;
  const Scalar _tmp68 = -_tmp45 + _tmp49;
  const Scalar _tmp69 = _tmp47 * _tmp64 -
                        _tmp54 * (-_tmp48 * _tmp64 - _tmp51 * _tmp67 + _tmp65 * _tmp68) -
                        _tmp55 * _tmp67 - _tmp64 * _tmp68;
  const Scalar _tmp70 = Scalar(1.0) / (_tmp69);
  const Scalar _tmp71 = _tmp38 * _tmp64 - _tmp40 * _tmp67 + _tmp58 * _tmp64 - _tmp61 * _tmp65;
  const Scalar _tmp72 = _tmp70 * _tmp71;
  const Scalar _tmp73 = Scalar(1.0) / (_tmp71);
  const Scalar _tmp74 = _tmp69 * _tmp73;
  const Scalar _tmp75 = _tmp56 + _tmp74 * (-_tmp40 * _tmp43 - _tmp56 * _tmp72);
  const Scalar _tmp76 = _tmp64 * _tmp70;
  const Scalar _tmp77 = _tmp66 * _tmp70;
  const Scalar _tmp78 = -_tmp75 * _tmp77 + Scalar(1.0);
  const Scalar _tmp79 = _tmp27 * _tmp42;
  const Scalar _tmp80 = _tmp31 + _tmp57;
  const Scalar _tmp81 = _tmp80 + position_vector(1, 0) + Scalar(8.3196563700000006);
  const Scalar _tmp82 = _tmp28 + _tmp60;
  const Scalar _tmp83 = _tmp82 + position_vector(0, 0) + Scalar(1.9874742000000001);
  const Scalar _tmp84 = std::pow(Scalar(std::pow(_tmp81, Scalar(2)) + std::pow(_tmp83, Scalar(2))),
                                 Scalar(Scalar(-1) / Scalar(2)));
  const Scalar _tmp85 = _tmp81 * _tmp84;
  const Scalar _tmp86 = _tmp37 * fh1;
  const Scalar _tmp87 = _tmp41 * _tmp42;
  const Scalar _tmp88 = -_tmp47 - _tmp54 * (_tmp48 + _tmp51 * _tmp87) + _tmp55 * _tmp87;
  const Scalar _tmp89 = _tmp74 * (-_tmp38 + _tmp40 * _tmp87 - _tmp72 * _tmp88) + _tmp88;
  const Scalar _tmp90 = -_tmp41 - _tmp77 * _tmp89;
  const Scalar _tmp91 = _tmp83 * _tmp84;
  const Scalar _tmp92 = Scalar(43.164000000000001) - fv1;
  const Scalar _tmp93 = _tmp52 * _tmp54 + _tmp53;
  const Scalar _tmp94 = 0;
  const Scalar _tmp95 = _tmp67 * _tmp94;
  const Scalar _tmp96 = _tmp37 * (-_tmp27 * _tmp95 + _tmp64 * _tmp94);
  const Scalar _tmp97 = Scalar(1.0) * _tmp73;
  const Scalar _tmp98 = fh1 * (-_tmp80 * _tmp91 + _tmp82 * _tmp85);
  const Scalar _tmp99 = _tmp42 * fh1;
  const Scalar _tmp100 = _tmp97 * _tmp98;
  const Scalar _tmp101 = _tmp92 * _tmp94;
  const Scalar _tmp102 = _tmp70 * fh1;

  // Output terms (1)
  Eigen::Matrix<Scalar, 4, 1> _res;

  _res(0, 0) = 0;
  _res(1, 0) =
      -_tmp96 * std::exp(_tmp37 * _tmp98 * (-_tmp27 * _tmp67 * _tmp97 + _tmp64 * _tmp97) +
                         _tmp85 * _tmp86 * (_tmp75 * _tmp76 + _tmp78 * _tmp79) +
                         _tmp86 * _tmp91 * (_tmp76 * _tmp89 + _tmp79 * _tmp90 + Scalar(1.0)) +
                         _tmp92 * _tmp96);
  _res(2, 0) = -_tmp95 * std::exp(_tmp100 * _tmp67 + _tmp101 * _tmp67 - _tmp78 * _tmp85 * _tmp99 -
                                  _tmp90 * _tmp91 * _tmp99);
  _res(3, 0) =
      _tmp94 * std::exp(-_tmp100 - _tmp101 - _tmp102 * _tmp75 * _tmp85 - _tmp102 * _tmp89 * _tmp91);

  return _res;
}  // NOLINT(readability/fn_size)

// NOLINTNEXTLINE(readability/fn_size)
}  // namespace sym
