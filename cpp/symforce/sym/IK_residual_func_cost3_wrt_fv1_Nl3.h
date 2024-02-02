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
 * Symbolic function: IK_residual_func_cost3_wrt_fv1_Nl3
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
Eigen::Matrix<Scalar, 4, 1> IkResidualFuncCost3WrtFv1Nl3(
    const Scalar fh1, const Scalar fv1, const sym::Rot3<Scalar>& DeltaRot,
    const Eigen::Matrix<Scalar, 3, 1>& position_vector, const sym::Rot3<Scalar>& Rot_init,
    const Scalar epsilon) {
  // Total ops: 288

  // Unused inputs
  (void)epsilon;

  // Input arrays
  const Eigen::Matrix<Scalar, 4, 1>& _DeltaRot = DeltaRot.Data();
  const Eigen::Matrix<Scalar, 4, 1>& _Rot_init = Rot_init.Data();

  // Intermediate terms (106)
  const Scalar _tmp0 = _DeltaRot[0] * _Rot_init[3] - _DeltaRot[1] * _Rot_init[2] +
                       _DeltaRot[2] * _Rot_init[1] + _DeltaRot[3] * _Rot_init[0];
  const Scalar _tmp1 = _DeltaRot[0] * _Rot_init[2] + _DeltaRot[1] * _Rot_init[3] -
                       _DeltaRot[2] * _Rot_init[0] + _DeltaRot[3] * _Rot_init[1];
  const Scalar _tmp2 = 2 * _tmp1;
  const Scalar _tmp3 = _tmp0 * _tmp2;
  const Scalar _tmp4 = -_DeltaRot[0] * _Rot_init[0] - _DeltaRot[1] * _Rot_init[1] -
                       _DeltaRot[2] * _Rot_init[2] + _DeltaRot[3] * _Rot_init[3];
  const Scalar _tmp5 = -_DeltaRot[0] * _Rot_init[1] + _DeltaRot[1] * _Rot_init[0] +
                       _DeltaRot[2] * _Rot_init[3] + _DeltaRot[3] * _Rot_init[2];
  const Scalar _tmp6 = 2 * _tmp5;
  const Scalar _tmp7 = _tmp4 * _tmp6;
  const Scalar _tmp8 = Scalar(0.20999999999999999) * _tmp3 + Scalar(0.20999999999999999) * _tmp7;
  const Scalar _tmp9 = _tmp1 * _tmp6;
  const Scalar _tmp10 = 2 * _tmp0 * _tmp4;
  const Scalar _tmp11 =
      Scalar(0.010999999999999999) * _tmp10 - Scalar(0.010999999999999999) * _tmp9;
  const Scalar _tmp12 = -2 * std::pow(_tmp0, Scalar(2));
  const Scalar _tmp13 = 1 - 2 * std::pow(_tmp5, Scalar(2));
  const Scalar _tmp14 = Scalar(0.20999999999999999) * _tmp12 + Scalar(0.20999999999999999) * _tmp13;
  const Scalar _tmp15 = _tmp11 + _tmp14;
  const Scalar _tmp16 = _tmp15 + _tmp8;
  const Scalar _tmp17 = Scalar(1.0) * _tmp16;
  const Scalar _tmp18 = -_tmp8;
  const Scalar _tmp19 = _tmp15 + _tmp18;
  const Scalar _tmp20 = Scalar(0.20999999999999999) * _tmp3 - Scalar(0.20999999999999999) * _tmp7;
  const Scalar _tmp21 = -2 * std::pow(_tmp1, Scalar(2));
  const Scalar _tmp22 = Scalar(0.20999999999999999) * _tmp13 + Scalar(0.20999999999999999) * _tmp21;
  const Scalar _tmp23 = _tmp0 * _tmp6;
  const Scalar _tmp24 = _tmp2 * _tmp4;
  const Scalar _tmp25 =
      -Scalar(0.010999999999999999) * _tmp23 - Scalar(0.010999999999999999) * _tmp24;
  const Scalar _tmp26 = -_tmp22 + _tmp25;
  const Scalar _tmp27 = _tmp20 + _tmp26;
  const Scalar _tmp28 = _tmp22 + _tmp25;
  const Scalar _tmp29 = _tmp20 + _tmp28;
  const Scalar _tmp30 = Scalar(1.0) * _tmp29;
  const Scalar _tmp31 = (-_tmp27 + _tmp30) / (-_tmp17 + _tmp19);
  const Scalar _tmp32 = _tmp19 + position_vector(1, 0) + Scalar(-4.8333311099999996);
  const Scalar _tmp33 = _tmp27 + position_vector(0, 0) + Scalar(1.79662371);
  const Scalar _tmp34 = std::pow(Scalar(std::pow(_tmp32, Scalar(2)) + std::pow(_tmp33, Scalar(2))),
                                 Scalar(Scalar(-1) / Scalar(2)));
  const Scalar _tmp35 = _tmp32 * _tmp34;
  const Scalar _tmp36 = _tmp16 + position_vector(1, 0) + Scalar(-4.7752063900000001);
  const Scalar _tmp37 = _tmp29 + position_vector(0, 0) + Scalar(-2.71799795);
  const Scalar _tmp38 = Scalar(1.0) / (_tmp37);
  const Scalar _tmp39 = _tmp36 * _tmp38;
  const Scalar _tmp40 = _tmp33 * _tmp34;
  const Scalar _tmp41 = Scalar(1.0) / (-_tmp35 + _tmp39 * _tmp40);
  const Scalar _tmp42 = Scalar(0.20999999999999999) * _tmp23 - Scalar(0.20999999999999999) * _tmp24;
  const Scalar _tmp43 = -Scalar(0.010999999999999999) * _tmp12 -
                        Scalar(0.010999999999999999) * _tmp21 + Scalar(-0.010999999999999999);
  const Scalar _tmp44 = Scalar(0.20999999999999999) * _tmp10 + Scalar(0.20999999999999999) * _tmp9;
  const Scalar _tmp45 = _tmp43 + _tmp44;
  const Scalar _tmp46 = -_tmp42 + _tmp45;
  const Scalar _tmp47 = _tmp42 + _tmp45;
  const Scalar _tmp48 = _tmp40 * _tmp47;
  const Scalar _tmp49 = _tmp35 * _tmp46 - _tmp39 * _tmp48;
  const Scalar _tmp50 = _tmp41 * _tmp49;
  const Scalar _tmp51 = -_tmp40 * _tmp46 + _tmp48;
  const Scalar _tmp52 = _tmp41 * _tmp51;
  const Scalar _tmp53 = Scalar(1.0) * _tmp31 * _tmp50 - Scalar(1.0) * _tmp52;
  const Scalar _tmp54 = _tmp11 - _tmp14;
  const Scalar _tmp55 = _tmp54 + _tmp8;
  const Scalar _tmp56 = _tmp55 + position_vector(1, 0) + Scalar(8.3888750099999996);
  const Scalar _tmp57 = -_tmp20;
  const Scalar _tmp58 = _tmp28 + _tmp57;
  const Scalar _tmp59 = _tmp58 + position_vector(0, 0) + Scalar(-2.5202214700000001);
  const Scalar _tmp60 = std::pow(Scalar(std::pow(_tmp56, Scalar(2)) + std::pow(_tmp59, Scalar(2))),
                                 Scalar(Scalar(-1) / Scalar(2)));
  const Scalar _tmp61 = _tmp59 * _tmp60;
  const Scalar _tmp62 = _tmp47 * _tmp61;
  const Scalar _tmp63 = _tmp42 + _tmp43 - _tmp44;
  const Scalar _tmp64 = _tmp56 * _tmp60;
  const Scalar _tmp65 = _tmp39 * _tmp61 - _tmp64;
  const Scalar _tmp66 = _tmp41 * _tmp65;
  const Scalar _tmp67 = -_tmp31 * (-_tmp39 * _tmp62 - _tmp49 * _tmp66 + _tmp63 * _tmp64) -
                        _tmp51 * _tmp66 - _tmp61 * _tmp63 + _tmp62;
  const Scalar _tmp68 = Scalar(1.0) / (_tmp67);
  const Scalar _tmp69 =
      std::sqrt(Scalar(std::pow(_tmp36, Scalar(2)) + std::pow(_tmp37, Scalar(2))));
  const Scalar _tmp70 = Scalar(1.0) / (_tmp69);
  const Scalar _tmp71 = _tmp38 * _tmp69;
  const Scalar _tmp72 = _tmp71 * (-_tmp16 * _tmp37 * _tmp70 + _tmp29 * _tmp36 * _tmp70);
  const Scalar _tmp73 = _tmp41 * (_tmp19 * _tmp40 - _tmp27 * _tmp35 + _tmp40 * _tmp72);
  const Scalar _tmp74 = _tmp55 * _tmp61 - _tmp58 * _tmp64 + _tmp61 * _tmp72 - _tmp65 * _tmp73;
  const Scalar _tmp75 = _tmp68 * _tmp74;
  const Scalar _tmp76 = Scalar(1.0) / (_tmp74);
  const Scalar _tmp77 = _tmp67 * _tmp76;
  const Scalar _tmp78 = _tmp53 + _tmp77 * (-_tmp53 * _tmp75 - Scalar(1.0) * _tmp73);
  const Scalar _tmp79 = _tmp65 * _tmp68;
  const Scalar _tmp80 = -_tmp78 * _tmp79 + Scalar(1.0);
  const Scalar _tmp81 = _tmp40 * _tmp41;
  const Scalar _tmp82 = _tmp61 * _tmp68;
  const Scalar _tmp83 = _tmp26 + _tmp57;
  const Scalar _tmp84 = _tmp83 + position_vector(0, 0) + Scalar(1.9874742000000001);
  const Scalar _tmp85 = _tmp18 + _tmp54;
  const Scalar _tmp86 = _tmp85 + position_vector(1, 0) + Scalar(8.3196563700000006);
  const Scalar _tmp87 = std::pow(Scalar(std::pow(_tmp84, Scalar(2)) + std::pow(_tmp86, Scalar(2))),
                                 Scalar(Scalar(-1) / Scalar(2)));
  const Scalar _tmp88 = _tmp86 * _tmp87;
  const Scalar _tmp89 = _tmp71 * fh1;
  const Scalar _tmp90 = Scalar(43.164000000000001) - fv1;
  const Scalar _tmp91 = _tmp17 * _tmp31 + _tmp30;
  const Scalar _tmp92 = 0;
  const Scalar _tmp93 = _tmp79 * _tmp92;
  const Scalar _tmp94 = _tmp68 * _tmp92;
  const Scalar _tmp95 = _tmp71 * (_tmp61 * _tmp94 - _tmp81 * _tmp93);
  const Scalar _tmp96 = -_tmp31 * (_tmp39 * _tmp47 + _tmp39 * _tmp50) + _tmp39 * _tmp52 - _tmp47;
  const Scalar _tmp97 = _tmp77 * (_tmp39 * _tmp73 - _tmp72 - _tmp75 * _tmp96) + _tmp96;
  const Scalar _tmp98 = -_tmp39 - _tmp79 * _tmp97;
  const Scalar _tmp99 = _tmp84 * _tmp87;
  const Scalar _tmp100 = Scalar(1.0) * _tmp76;
  const Scalar _tmp101 = fh1 * (_tmp83 * _tmp88 - _tmp85 * _tmp99);
  const Scalar _tmp102 = _tmp41 * fh1;
  const Scalar _tmp103 = _tmp41 * _tmp93;
  const Scalar _tmp104 = _tmp100 * _tmp101;
  const Scalar _tmp105 = _tmp68 * fh1;

  // Output terms (1)
  Eigen::Matrix<Scalar, 4, 1> _res;

  _res(0, 0) = 0;
  _res(1, 0) =
      -_tmp95 * std::exp(_tmp101 * _tmp71 * (-_tmp100 * _tmp40 * _tmp66 + _tmp100 * _tmp61) +
                         _tmp88 * _tmp89 * (_tmp78 * _tmp82 + _tmp80 * _tmp81) +
                         _tmp89 * _tmp99 * (_tmp81 * _tmp98 + _tmp82 * _tmp97 + Scalar(1.0)) +
                         _tmp90 * _tmp95);
  _res(2, 0) = -_tmp103 * std::exp(-_tmp102 * _tmp80 * _tmp88 - _tmp102 * _tmp98 * _tmp99 +
                                   _tmp103 * _tmp90 + _tmp104 * _tmp66);
  _res(3, 0) = _tmp94 * std::exp(-_tmp104 - _tmp105 * _tmp78 * _tmp88 - _tmp105 * _tmp97 * _tmp99 -
                                 _tmp90 * _tmp94);

  return _res;
}  // NOLINT(readability/fn_size)

// NOLINTNEXTLINE(readability/fn_size)
}  // namespace sym