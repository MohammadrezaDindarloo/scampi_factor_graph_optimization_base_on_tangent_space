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
 * Symbolic function: FK_residual_func_cost3_wrt_fh1
 *
 * Args:
 *     fh1: Scalar
 *     fv1: Scalar
 *     DeltaRot: Rot3
 *     position_vector: Matrix31
 *     Rot_init: Rot3
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
Eigen::Matrix<Scalar, 4, 1> FkResidualFuncCost3WrtFh1(
    const Scalar fh1, const Scalar fv1, const sym::Rot3<Scalar>& DeltaRot,
    const Eigen::Matrix<Scalar, 3, 1>& position_vector, const sym::Rot3<Scalar>& Rot_init,
    const Scalar lc0, const Scalar lc1, const Scalar lc2, const Scalar lc3, const Scalar epsilon) {
  // Total ops: 303

  // Unused inputs
  (void)lc0;
  (void)lc1;
  (void)lc2;
  (void)lc3;
  (void)epsilon;

  // Input arrays
  const Eigen::Matrix<Scalar, 4, 1>& _DeltaRot = DeltaRot.Data();
  const Eigen::Matrix<Scalar, 4, 1>& _Rot_init = Rot_init.Data();

  // Intermediate terms (109)
  const Scalar _tmp0 = _DeltaRot[0] * _Rot_init[2] + _DeltaRot[1] * _Rot_init[3] -
                       _DeltaRot[2] * _Rot_init[0] + _DeltaRot[3] * _Rot_init[1];
  const Scalar _tmp1 = -_DeltaRot[0] * _Rot_init[1] + _DeltaRot[1] * _Rot_init[0] +
                       _DeltaRot[2] * _Rot_init[3] + _DeltaRot[3] * _Rot_init[2];
  const Scalar _tmp2 = 2 * _tmp1;
  const Scalar _tmp3 = _tmp0 * _tmp2;
  const Scalar _tmp4 = -_DeltaRot[0] * _Rot_init[0] - _DeltaRot[1] * _Rot_init[1] -
                       _DeltaRot[2] * _Rot_init[2] + _DeltaRot[3] * _Rot_init[3];
  const Scalar _tmp5 = _DeltaRot[0] * _Rot_init[3] - _DeltaRot[1] * _Rot_init[2] +
                       _DeltaRot[2] * _Rot_init[1] + _DeltaRot[3] * _Rot_init[0];
  const Scalar _tmp6 = 2 * _tmp5;
  const Scalar _tmp7 = _tmp4 * _tmp6;
  const Scalar _tmp8 = Scalar(0.20999999999999999) * _tmp3 + Scalar(0.20999999999999999) * _tmp7;
  const Scalar _tmp9 = -2 * std::pow(_tmp0, Scalar(2));
  const Scalar _tmp10 = -2 * std::pow(_tmp5, Scalar(2));
  const Scalar _tmp11 = -Scalar(0.010999999999999999) * _tmp10 -
                        Scalar(0.010999999999999999) * _tmp9 + Scalar(-0.010999999999999999);
  const Scalar _tmp12 = _tmp2 * _tmp5;
  const Scalar _tmp13 = 2 * _tmp0 * _tmp4;
  const Scalar _tmp14 = Scalar(0.20999999999999999) * _tmp12 - Scalar(0.20999999999999999) * _tmp13;
  const Scalar _tmp15 = _tmp11 + _tmp14;
  const Scalar _tmp16 = _tmp15 - _tmp8;
  const Scalar _tmp17 = _tmp0 * _tmp6;
  const Scalar _tmp18 = _tmp2 * _tmp4;
  const Scalar _tmp19 = Scalar(0.20999999999999999) * _tmp17 + Scalar(0.20999999999999999) * _tmp18;
  const Scalar _tmp20 = 1 - 2 * std::pow(_tmp1, Scalar(2));
  const Scalar _tmp21 = Scalar(0.20999999999999999) * _tmp10 + Scalar(0.20999999999999999) * _tmp20;
  const Scalar _tmp22 =
      -Scalar(0.010999999999999999) * _tmp3 + Scalar(0.010999999999999999) * _tmp7;
  const Scalar _tmp23 = _tmp21 + _tmp22;
  const Scalar _tmp24 = _tmp19 + _tmp23;
  const Scalar _tmp25 = _tmp24 + position_vector(1, 0) + Scalar(-4.7752063900000001);
  const Scalar _tmp26 = Scalar(0.20999999999999999) * _tmp17 - Scalar(0.20999999999999999) * _tmp18;
  const Scalar _tmp27 = Scalar(0.20999999999999999) * _tmp20 + Scalar(0.20999999999999999) * _tmp9;
  const Scalar _tmp28 =
      -Scalar(0.010999999999999999) * _tmp12 - Scalar(0.010999999999999999) * _tmp13;
  const Scalar _tmp29 = _tmp27 + _tmp28;
  const Scalar _tmp30 = _tmp26 + _tmp29;
  const Scalar _tmp31 = _tmp30 + position_vector(0, 0) + Scalar(-2.71799795);
  const Scalar _tmp32 = std::pow(Scalar(std::pow(_tmp25, Scalar(2)) + std::pow(_tmp31, Scalar(2))),
                                 Scalar(Scalar(-1) / Scalar(2)));
  const Scalar _tmp33 = _tmp31 * _tmp32;
  const Scalar _tmp34 = _tmp16 * _tmp33;
  const Scalar _tmp35 = _tmp15 + _tmp8;
  const Scalar _tmp36 = -_tmp33 * _tmp35 + _tmp34;
  const Scalar _tmp37 = -_tmp26;
  const Scalar _tmp38 = _tmp29 + _tmp37;
  const Scalar _tmp39 = _tmp38 + position_vector(0, 0) + Scalar(-2.5202214700000001);
  const Scalar _tmp40 = Scalar(1.0) / (_tmp39);
  const Scalar _tmp41 = -_tmp21 + _tmp22;
  const Scalar _tmp42 = _tmp19 + _tmp41;
  const Scalar _tmp43 = _tmp42 + position_vector(1, 0) + Scalar(8.3888750099999996);
  const Scalar _tmp44 = _tmp40 * _tmp43;
  const Scalar _tmp45 = _tmp25 * _tmp32;
  const Scalar _tmp46 = Scalar(1.0) / (_tmp33 * _tmp44 - _tmp45);
  const Scalar _tmp47 = _tmp44 * _tmp46;
  const Scalar _tmp48 = -_tmp34 * _tmp44 + _tmp35 * _tmp45;
  const Scalar _tmp49 = _tmp16 * _tmp44;
  const Scalar _tmp50 = Scalar(1.0) * _tmp42;
  const Scalar _tmp51 = Scalar(1.0) * _tmp38;
  const Scalar _tmp52 = (-_tmp30 + _tmp51) / (_tmp24 - _tmp50);
  const Scalar _tmp53 = -_tmp16 + _tmp36 * _tmp47 - _tmp52 * (_tmp47 * _tmp48 + _tmp49);
  const Scalar _tmp54 = _tmp11 - _tmp14 + _tmp8;
  const Scalar _tmp55 = -_tmp27 + _tmp28;
  const Scalar _tmp56 = _tmp26 + _tmp55;
  const Scalar _tmp57 = _tmp56 + position_vector(0, 0) + Scalar(1.79662371);
  const Scalar _tmp58 = -_tmp19;
  const Scalar _tmp59 = _tmp23 + _tmp58;
  const Scalar _tmp60 = _tmp59 + position_vector(1, 0) + Scalar(-4.8333311099999996);
  const Scalar _tmp61 = std::pow(Scalar(std::pow(_tmp57, Scalar(2)) + std::pow(_tmp60, Scalar(2))),
                                 Scalar(Scalar(-1) / Scalar(2)));
  const Scalar _tmp62 = _tmp57 * _tmp61;
  const Scalar _tmp63 = _tmp60 * _tmp61;
  const Scalar _tmp64 = _tmp44 * _tmp62 - _tmp63;
  const Scalar _tmp65 = _tmp46 * _tmp64;
  const Scalar _tmp66 = _tmp16 * _tmp62 - _tmp36 * _tmp65 -
                        _tmp52 * (-_tmp48 * _tmp65 - _tmp49 * _tmp62 + _tmp54 * _tmp63) -
                        _tmp54 * _tmp62;
  const Scalar _tmp67 = Scalar(1.0) / (_tmp66);
  const Scalar _tmp68 =
      std::sqrt(Scalar(std::pow(_tmp39, Scalar(2)) + std::pow(_tmp43, Scalar(2))));
  const Scalar _tmp69 = Scalar(1.0) / (_tmp68);
  const Scalar _tmp70 = _tmp40 * _tmp68;
  const Scalar _tmp71 = _tmp70 * (_tmp38 * _tmp43 * _tmp69 - _tmp39 * _tmp42 * _tmp69);
  const Scalar _tmp72 = _tmp24 * _tmp33 - _tmp30 * _tmp45 + _tmp33 * _tmp71;
  const Scalar _tmp73 = -_tmp56 * _tmp63 + _tmp59 * _tmp62 + _tmp62 * _tmp71 - _tmp65 * _tmp72;
  const Scalar _tmp74 = _tmp67 * _tmp73;
  const Scalar _tmp75 = Scalar(1.0) / (_tmp73);
  const Scalar _tmp76 = _tmp66 * _tmp75;
  const Scalar _tmp77 = _tmp53 + _tmp76 * (_tmp47 * _tmp72 - _tmp53 * _tmp74 - _tmp71);
  const Scalar _tmp78 = _tmp62 * _tmp67;
  const Scalar _tmp79 = _tmp64 * _tmp67;
  const Scalar _tmp80 = -_tmp44 - _tmp77 * _tmp79;
  const Scalar _tmp81 = _tmp33 * _tmp46;
  const Scalar _tmp82 = _tmp37 + _tmp55;
  const Scalar _tmp83 = _tmp82 + position_vector(0, 0) + Scalar(1.9874742000000001);
  const Scalar _tmp84 = _tmp41 + _tmp58;
  const Scalar _tmp85 = _tmp84 + position_vector(1, 0) + Scalar(8.3196563700000006);
  const Scalar _tmp86 = std::pow(Scalar(std::pow(_tmp83, Scalar(2)) + std::pow(_tmp85, Scalar(2))),
                                 Scalar(Scalar(-1) / Scalar(2)));
  const Scalar _tmp87 = _tmp83 * _tmp86;
  const Scalar _tmp88 = _tmp70 * _tmp87 * (_tmp77 * _tmp78 + _tmp80 * _tmp81 + Scalar(1.0));
  const Scalar _tmp89 = Scalar(43.164000000000001) - fv1;
  const Scalar _tmp90 = _tmp50 * _tmp52 + _tmp51;
  const Scalar _tmp91 = 0;
  const Scalar _tmp92 = _tmp79 * _tmp91;
  const Scalar _tmp93 = _tmp67 * _tmp91;
  const Scalar _tmp94 = Scalar(1.0) * _tmp46;
  const Scalar _tmp95 = -_tmp36 * _tmp94 + _tmp48 * _tmp52 * _tmp94;
  const Scalar _tmp96 = _tmp76 * (-_tmp72 * _tmp94 - _tmp74 * _tmp95) + _tmp95;
  const Scalar _tmp97 = -_tmp79 * _tmp96 + Scalar(1.0);
  const Scalar _tmp98 = _tmp85 * _tmp86;
  const Scalar _tmp99 = _tmp70 * _tmp98 * (_tmp78 * _tmp96 + _tmp81 * _tmp97);
  const Scalar _tmp100 = _tmp82 * _tmp98 - _tmp84 * _tmp87;
  const Scalar _tmp101 = Scalar(1.0) * _tmp75;
  const Scalar _tmp102 = _tmp100 * _tmp70 * (-_tmp101 * _tmp33 * _tmp65 + _tmp101 * _tmp62);
  const Scalar _tmp103 = _tmp100 * _tmp101;
  const Scalar _tmp104 = _tmp103 * fh1;
  const Scalar _tmp105 = _tmp46 * _tmp80 * _tmp87;
  const Scalar _tmp106 = _tmp46 * _tmp97 * _tmp98;
  const Scalar _tmp107 = _tmp67 * _tmp96 * _tmp98;
  const Scalar _tmp108 = _tmp67 * _tmp77 * _tmp87;

  // Output terms (1)
  Eigen::Matrix<Scalar, 4, 1> _res;

  _res(0, 0) = -std::exp(-fh1);
  _res(1, 0) = -(-_tmp102 - _tmp88 - _tmp99) *
               std::exp(_tmp102 * fh1 + _tmp70 * _tmp89 * (_tmp62 * _tmp93 - _tmp81 * _tmp92) +
                        _tmp88 * fh1 + _tmp99 * fh1);
  _res(2, 0) =
      -(-_tmp103 * _tmp65 + _tmp105 + _tmp106) *
      std::exp(_tmp104 * _tmp65 - _tmp105 * fh1 - _tmp106 * fh1 + _tmp46 * _tmp89 * _tmp92);
  _res(3, 0) = -(_tmp103 + _tmp107 + _tmp108) *
               std::exp(-_tmp104 - _tmp107 * fh1 - _tmp108 * fh1 - _tmp89 * _tmp93);

  return _res;
}  // NOLINT(readability/fn_size)

// NOLINTNEXTLINE(readability/fn_size)
}  // namespace sym