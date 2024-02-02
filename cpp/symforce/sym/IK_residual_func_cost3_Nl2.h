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
 * Symbolic function: IK_residual_func_cost3_Nl2
 *
 * Args:
 *     fh1: Scalar
 *     fv1: Scalar
 *     DeltaRot: Rot3
 *     position_vector: Matrix31
 *     Rot_init: Rot3
 *     epsilon: Scalar
 *
 * Outputs:
 *     res: Matrix41
 */
template <typename Scalar>
Eigen::Matrix<Scalar, 4, 1> IkResidualFuncCost3Nl2(
    const Scalar fh1, const Scalar fv1, const sym::Rot3<Scalar>& DeltaRot,
    const Eigen::Matrix<Scalar, 3, 1>& position_vector, const sym::Rot3<Scalar>& Rot_init,
    const Scalar epsilon) {
  // Total ops: 284

  // Unused inputs
  (void)epsilon;

  // Input arrays
  const Eigen::Matrix<Scalar, 4, 1>& _DeltaRot = DeltaRot.Data();
  const Eigen::Matrix<Scalar, 4, 1>& _Rot_init = Rot_init.Data();

  // Intermediate terms (103)
  const Scalar _tmp0 = _DeltaRot[0] * _Rot_init[2] + _DeltaRot[1] * _Rot_init[3] -
                       _DeltaRot[2] * _Rot_init[0] + _DeltaRot[3] * _Rot_init[1];
  const Scalar _tmp1 = _DeltaRot[0] * _Rot_init[3] - _DeltaRot[1] * _Rot_init[2] +
                       _DeltaRot[2] * _Rot_init[1] + _DeltaRot[3] * _Rot_init[0];
  const Scalar _tmp2 = 2 * _tmp0 * _tmp1;
  const Scalar _tmp3 = -_DeltaRot[0] * _Rot_init[1] + _DeltaRot[1] * _Rot_init[0] +
                       _DeltaRot[2] * _Rot_init[3] + _DeltaRot[3] * _Rot_init[2];
  const Scalar _tmp4 = -2 * _DeltaRot[0] * _Rot_init[0] - 2 * _DeltaRot[1] * _Rot_init[1] -
                       2 * _DeltaRot[2] * _Rot_init[2] + 2 * _DeltaRot[3] * _Rot_init[3];
  const Scalar _tmp5 = _tmp3 * _tmp4;
  const Scalar _tmp6 = Scalar(0.20999999999999999) * _tmp2 - Scalar(0.20999999999999999) * _tmp5;
  const Scalar _tmp7 = -2 * std::pow(_tmp3, Scalar(2));
  const Scalar _tmp8 = 1 - 2 * std::pow(_tmp0, Scalar(2));
  const Scalar _tmp9 = Scalar(0.20999999999999999) * _tmp7 + Scalar(0.20999999999999999) * _tmp8;
  const Scalar _tmp10 = 2 * _tmp3;
  const Scalar _tmp11 = _tmp1 * _tmp10;
  const Scalar _tmp12 = _tmp0 * _tmp4;
  const Scalar _tmp13 =
      -Scalar(0.010999999999999999) * _tmp11 - Scalar(0.010999999999999999) * _tmp12;
  const Scalar _tmp14 = _tmp13 - _tmp9;
  const Scalar _tmp15 = _tmp14 + _tmp6;
  const Scalar _tmp16 = _tmp15 + position_vector(0, 0) + Scalar(1.79662371);
  const Scalar _tmp17 = -2 * std::pow(_tmp1, Scalar(2));
  const Scalar _tmp18 = Scalar(0.20999999999999999) * _tmp17 + Scalar(0.20999999999999999) * _tmp7 +
                        Scalar(0.20999999999999999);
  const Scalar _tmp19 = _tmp0 * _tmp10;
  const Scalar _tmp20 = _tmp1 * _tmp4;
  const Scalar _tmp21 =
      -Scalar(0.010999999999999999) * _tmp19 + Scalar(0.010999999999999999) * _tmp20;
  const Scalar _tmp22 = Scalar(0.20999999999999999) * _tmp2 + Scalar(0.20999999999999999) * _tmp5;
  const Scalar _tmp23 = _tmp21 - _tmp22;
  const Scalar _tmp24 = _tmp18 + _tmp23;
  const Scalar _tmp25 = _tmp24 + position_vector(1, 0) + Scalar(-4.8333311099999996);
  const Scalar _tmp26 = std::pow(Scalar(std::pow(_tmp16, Scalar(2)) + std::pow(_tmp25, Scalar(2))),
                                 Scalar(Scalar(-1) / Scalar(2)));
  const Scalar _tmp27 = _tmp16 * _tmp26;
  const Scalar _tmp28 = _tmp21 + _tmp22;
  const Scalar _tmp29 = _tmp18 + _tmp28;
  const Scalar _tmp30 = _tmp29 + position_vector(1, 0) + Scalar(-4.7752063900000001);
  const Scalar _tmp31 = _tmp13 + _tmp9;
  const Scalar _tmp32 = _tmp31 + _tmp6;
  const Scalar _tmp33 = _tmp32 + position_vector(0, 0) + Scalar(-2.71799795);
  const Scalar _tmp34 =
      std::sqrt(Scalar(std::pow(_tmp30, Scalar(2)) + std::pow(_tmp33, Scalar(2))));
  const Scalar _tmp35 = Scalar(1.0) / (_tmp34);
  const Scalar _tmp36 = Scalar(1.0) / (_tmp33);
  const Scalar _tmp37 = _tmp34 * _tmp36;
  const Scalar _tmp38 = _tmp37 * (-_tmp29 * _tmp33 * _tmp35 + _tmp30 * _tmp32 * _tmp35);
  const Scalar _tmp39 = -_tmp6;
  const Scalar _tmp40 = _tmp31 + _tmp39;
  const Scalar _tmp41 = -_tmp18;
  const Scalar _tmp42 = _tmp28 + _tmp41;
  const Scalar _tmp43 = _tmp42 + position_vector(1, 0) + Scalar(8.3888750099999996);
  const Scalar _tmp44 = _tmp40 + position_vector(0, 0) + Scalar(-2.5202214700000001);
  const Scalar _tmp45 = std::pow(Scalar(std::pow(_tmp43, Scalar(2)) + std::pow(_tmp44, Scalar(2))),
                                 Scalar(Scalar(-1) / Scalar(2)));
  const Scalar _tmp46 = _tmp43 * _tmp45;
  const Scalar _tmp47 = _tmp44 * _tmp45;
  const Scalar _tmp48 = _tmp38 * _tmp47 - _tmp40 * _tmp46 + _tmp42 * _tmp47;
  const Scalar _tmp49 = _tmp30 * _tmp36;
  const Scalar _tmp50 = Scalar(1.0) / (-_tmp46 + _tmp47 * _tmp49);
  const Scalar _tmp51 = _tmp25 * _tmp26;
  const Scalar _tmp52 = _tmp27 * _tmp49 - _tmp51;
  const Scalar _tmp53 = _tmp50 * _tmp52;
  const Scalar _tmp54 = -_tmp15 * _tmp51 + _tmp24 * _tmp27 + _tmp27 * _tmp38 - _tmp48 * _tmp53;
  const Scalar _tmp55 = Scalar(1.0) / (_tmp54);
  const Scalar _tmp56 = Scalar(1.0) * _tmp55;
  const Scalar _tmp57 = _tmp47 * _tmp53;
  const Scalar _tmp58 = _tmp14 + _tmp39;
  const Scalar _tmp59 = _tmp58 + position_vector(0, 0) + Scalar(1.9874742000000001);
  const Scalar _tmp60 = _tmp23 + _tmp41;
  const Scalar _tmp61 = _tmp60 + position_vector(1, 0) + Scalar(8.3196563700000006);
  const Scalar _tmp62 = std::pow(Scalar(std::pow(_tmp59, Scalar(2)) + std::pow(_tmp61, Scalar(2))),
                                 Scalar(Scalar(-1) / Scalar(2)));
  const Scalar _tmp63 = _tmp61 * _tmp62;
  const Scalar _tmp64 = _tmp59 * _tmp62;
  const Scalar _tmp65 = fh1 * (_tmp58 * _tmp63 - _tmp60 * _tmp64);
  const Scalar _tmp66 = Scalar(0.20999999999999999) * _tmp11 - Scalar(0.20999999999999999) * _tmp12;
  const Scalar _tmp67 =
      -Scalar(0.010999999999999999) * _tmp17 - Scalar(0.010999999999999999) * _tmp8;
  const Scalar _tmp68 = Scalar(0.20999999999999999) * _tmp19 + Scalar(0.20999999999999999) * _tmp20;
  const Scalar _tmp69 = _tmp67 + _tmp68;
  const Scalar _tmp70 = _tmp66 + _tmp69;
  const Scalar _tmp71 = _tmp47 * _tmp70;
  const Scalar _tmp72 = _tmp66 + _tmp67 - _tmp68;
  const Scalar _tmp73 = _tmp46 * _tmp72 - _tmp49 * _tmp71;
  const Scalar _tmp74 = _tmp49 * _tmp50;
  const Scalar _tmp75 = Scalar(1.0) * _tmp29;
  const Scalar _tmp76 = Scalar(1.0) * _tmp32;
  const Scalar _tmp77 = (-_tmp40 + _tmp76) / (_tmp42 - _tmp75);
  const Scalar _tmp78 = -_tmp47 * _tmp72 + _tmp71;
  const Scalar _tmp79 = -_tmp70 + _tmp74 * _tmp78 - _tmp77 * (_tmp49 * _tmp70 + _tmp73 * _tmp74);
  const Scalar _tmp80 = -_tmp66 + _tmp69;
  const Scalar _tmp81 = _tmp27 * _tmp70;
  const Scalar _tmp82 = -_tmp27 * _tmp80 - _tmp53 * _tmp78 -
                        _tmp77 * (-_tmp49 * _tmp81 + _tmp51 * _tmp80 - _tmp53 * _tmp73) + _tmp81;
  const Scalar _tmp83 = Scalar(1.0) / (_tmp82);
  const Scalar _tmp84 = _tmp54 * _tmp83;
  const Scalar _tmp85 = _tmp55 * _tmp82;
  const Scalar _tmp86 = _tmp79 + _tmp85 * (-_tmp38 + _tmp48 * _tmp74 - _tmp79 * _tmp84);
  const Scalar _tmp87 = _tmp52 * _tmp83;
  const Scalar _tmp88 = -_tmp49 - _tmp86 * _tmp87;
  const Scalar _tmp89 = _tmp47 * _tmp50;
  const Scalar _tmp90 = _tmp27 * _tmp83;
  const Scalar _tmp91 = _tmp37 * fh1;
  const Scalar _tmp92 = Scalar(1.0) * _tmp50;
  const Scalar _tmp93 = _tmp73 * _tmp77 * _tmp92 - _tmp78 * _tmp92;
  const Scalar _tmp94 = _tmp85 * (-_tmp48 * _tmp92 - _tmp84 * _tmp93) + _tmp93;
  const Scalar _tmp95 = -_tmp87 * _tmp94 + Scalar(1.0);
  const Scalar _tmp96 = Scalar(43.164000000000001) - fv1;
  const Scalar _tmp97 = _tmp75 * _tmp77 + _tmp76;
  const Scalar _tmp98 = 0;
  const Scalar _tmp99 = _tmp50 * fh1;
  const Scalar _tmp100 = _tmp96 * _tmp98;
  const Scalar _tmp101 = _tmp56 * _tmp65;
  const Scalar _tmp102 = _tmp83 * fh1;

  // Output terms (1)
  Eigen::Matrix<Scalar, 4, 1> _res;

  _res(0, 0) = std::exp(-fh1);
  _res(1, 0) = std::exp(_tmp37 * _tmp65 * (_tmp27 * _tmp56 - _tmp56 * _tmp57) +
                        _tmp37 * _tmp96 * (_tmp27 * _tmp98 - _tmp57 * _tmp98) +
                        _tmp63 * _tmp91 * (_tmp89 * _tmp95 + _tmp90 * _tmp94) +
                        _tmp64 * _tmp91 * (_tmp86 * _tmp90 + _tmp88 * _tmp89 + Scalar(1.0)));
  _res(2, 0) = std::exp(_tmp100 * _tmp53 + _tmp101 * _tmp53 - _tmp63 * _tmp95 * _tmp99 -
                        _tmp64 * _tmp88 * _tmp99);
  _res(3, 0) = std::exp(-_tmp100 - _tmp101 - _tmp102 * _tmp63 * _tmp94 - _tmp102 * _tmp64 * _tmp86);

  return _res;
}  // NOLINT(readability/fn_size)

// NOLINTNEXTLINE(readability/fn_size)
}  // namespace sym
