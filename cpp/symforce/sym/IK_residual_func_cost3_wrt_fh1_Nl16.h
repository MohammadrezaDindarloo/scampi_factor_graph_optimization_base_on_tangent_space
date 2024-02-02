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
 * Symbolic function: IK_residual_func_cost3_wrt_fh1_Nl16
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
Eigen::Matrix<Scalar, 4, 1> IkResidualFuncCost3WrtFh1Nl16(
    const Scalar fh1, const Scalar fv1, const sym::Rot3<Scalar>& DeltaRot,
    const Eigen::Matrix<Scalar, 3, 1>& position_vector, const sym::Rot3<Scalar>& Rot_init,
    const Scalar epsilon) {
  // Total ops: 304

  // Unused inputs
  (void)epsilon;

  // Input arrays
  const Eigen::Matrix<Scalar, 4, 1>& _DeltaRot = DeltaRot.Data();
  const Eigen::Matrix<Scalar, 4, 1>& _Rot_init = Rot_init.Data();

  // Intermediate terms (108)
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
  const Scalar _tmp6 = Scalar(0.20999999999999999) * _tmp2 + Scalar(0.20999999999999999) * _tmp5;
  const Scalar _tmp7 = -_tmp6;
  const Scalar _tmp8 = 2 * _tmp3;
  const Scalar _tmp9 = _tmp0 * _tmp8;
  const Scalar _tmp10 = _tmp1 * _tmp4;
  const Scalar _tmp11 =
      Scalar(0.010999999999999999) * _tmp10 - Scalar(0.010999999999999999) * _tmp9;
  const Scalar _tmp12 = -2 * std::pow(_tmp1, Scalar(2));
  const Scalar _tmp13 = -2 * std::pow(_tmp3, Scalar(2));
  const Scalar _tmp14 = Scalar(0.20999999999999999) * _tmp12 +
                        Scalar(0.20999999999999999) * _tmp13 + Scalar(0.20999999999999999);
  const Scalar _tmp15 = _tmp11 + _tmp14;
  const Scalar _tmp16 = _tmp15 + _tmp7;
  const Scalar _tmp17 = _tmp16 + position_vector(1, 0) + Scalar(-4.8333311099999996);
  const Scalar _tmp18 = Scalar(0.20999999999999999) * _tmp2 - Scalar(0.20999999999999999) * _tmp5;
  const Scalar _tmp19 = 1 - 2 * std::pow(_tmp0, Scalar(2));
  const Scalar _tmp20 = Scalar(0.20999999999999999) * _tmp13 + Scalar(0.20999999999999999) * _tmp19;
  const Scalar _tmp21 = _tmp1 * _tmp8;
  const Scalar _tmp22 = _tmp0 * _tmp4;
  const Scalar _tmp23 =
      -Scalar(0.010999999999999999) * _tmp21 - Scalar(0.010999999999999999) * _tmp22;
  const Scalar _tmp24 = -_tmp20 + _tmp23;
  const Scalar _tmp25 = _tmp18 + _tmp24;
  const Scalar _tmp26 = _tmp25 + position_vector(0, 0) + Scalar(1.79662371);
  const Scalar _tmp27 = Scalar(1.0) / (_tmp26);
  const Scalar _tmp28 = _tmp17 * _tmp27;
  const Scalar _tmp29 = -_tmp18;
  const Scalar _tmp30 = _tmp24 + _tmp29;
  const Scalar _tmp31 = _tmp30 + position_vector(0, 0) + Scalar(1.9874742000000001);
  const Scalar _tmp32 = _tmp11 - _tmp14;
  const Scalar _tmp33 = _tmp32 + _tmp7;
  const Scalar _tmp34 = _tmp33 + position_vector(1, 0) + Scalar(8.3196563700000006);
  const Scalar _tmp35 = std::pow(Scalar(std::pow(_tmp31, Scalar(2)) + std::pow(_tmp34, Scalar(2))),
                                 Scalar(Scalar(-1) / Scalar(2)));
  const Scalar _tmp36 = _tmp31 * _tmp35;
  const Scalar _tmp37 = _tmp34 * _tmp35;
  const Scalar _tmp38 = Scalar(1.0) / (_tmp28 * _tmp36 - _tmp37);
  const Scalar _tmp39 =
      std::sqrt(Scalar(std::pow(_tmp17, Scalar(2)) + std::pow(_tmp26, Scalar(2))));
  const Scalar _tmp40 = Scalar(1.0) / (_tmp39);
  const Scalar _tmp41 = _tmp27 * _tmp39;
  const Scalar _tmp42 = _tmp41 * (-_tmp16 * _tmp26 * _tmp40 + _tmp17 * _tmp25 * _tmp40);
  const Scalar _tmp43 = -_tmp30 * _tmp37 + _tmp33 * _tmp36 + _tmp36 * _tmp42;
  const Scalar _tmp44 = _tmp38 * _tmp43;
  const Scalar _tmp45 = Scalar(1.0) * _tmp16;
  const Scalar _tmp46 = Scalar(1.0) * _tmp25;
  const Scalar _tmp47 = (-_tmp30 + _tmp46) / (_tmp33 - _tmp45);
  const Scalar _tmp48 = Scalar(0.20999999999999999) * _tmp10 + Scalar(0.20999999999999999) * _tmp9;
  const Scalar _tmp49 = -_tmp48;
  const Scalar _tmp50 =
      -Scalar(0.010999999999999999) * _tmp12 - Scalar(0.010999999999999999) * _tmp19;
  const Scalar _tmp51 = Scalar(0.20999999999999999) * _tmp21 - Scalar(0.20999999999999999) * _tmp22;
  const Scalar _tmp52 = _tmp50 - _tmp51;
  const Scalar _tmp53 = _tmp49 + _tmp52;
  const Scalar _tmp54 = _tmp48 + _tmp52;
  const Scalar _tmp55 = _tmp28 * _tmp54;
  const Scalar _tmp56 = -_tmp36 * _tmp55 + _tmp37 * _tmp53;
  const Scalar _tmp57 = _tmp38 * _tmp56;
  const Scalar _tmp58 = _tmp38 * (-_tmp36 * _tmp53 + _tmp36 * _tmp54);
  const Scalar _tmp59 = Scalar(1.0) * _tmp47 * _tmp57 - Scalar(1.0) * _tmp58;
  const Scalar _tmp60 = _tmp20 + _tmp23;
  const Scalar _tmp61 = _tmp29 + _tmp60;
  const Scalar _tmp62 = _tmp32 + _tmp6;
  const Scalar _tmp63 = _tmp62 + position_vector(1, 0) + Scalar(8.3888750099999996);
  const Scalar _tmp64 = _tmp61 + position_vector(0, 0) + Scalar(-2.5202214700000001);
  const Scalar _tmp65 = std::pow(Scalar(std::pow(_tmp63, Scalar(2)) + std::pow(_tmp64, Scalar(2))),
                                 Scalar(Scalar(-1) / Scalar(2)));
  const Scalar _tmp66 = _tmp63 * _tmp65;
  const Scalar _tmp67 = _tmp64 * _tmp65;
  const Scalar _tmp68 = _tmp28 * _tmp67 - _tmp66;
  const Scalar _tmp69 = _tmp38 * _tmp68;
  const Scalar _tmp70 = _tmp42 * _tmp67 - _tmp43 * _tmp69 - _tmp61 * _tmp66 + _tmp62 * _tmp67;
  const Scalar _tmp71 = _tmp54 * _tmp67;
  const Scalar _tmp72 = _tmp49 + _tmp50 + _tmp51;
  const Scalar _tmp73 = -_tmp47 * (-_tmp28 * _tmp71 - _tmp56 * _tmp69 + _tmp66 * _tmp72) -
                        _tmp58 * _tmp68 - _tmp67 * _tmp72 + _tmp71;
  const Scalar _tmp74 = Scalar(1.0) / (_tmp73);
  const Scalar _tmp75 = _tmp70 * _tmp74;
  const Scalar _tmp76 = Scalar(1.0) / (_tmp70);
  const Scalar _tmp77 = _tmp73 * _tmp76;
  const Scalar _tmp78 = _tmp59 + _tmp77 * (-Scalar(1.0) * _tmp44 - _tmp59 * _tmp75);
  const Scalar _tmp79 = _tmp67 * _tmp74;
  const Scalar _tmp80 = _tmp68 * _tmp74;
  const Scalar _tmp81 = _tmp38 * (-_tmp78 * _tmp80 + Scalar(1.0));
  const Scalar _tmp82 = _tmp15 + _tmp6;
  const Scalar _tmp83 = _tmp82 + position_vector(1, 0) + Scalar(-4.7752063900000001);
  const Scalar _tmp84 = _tmp18 + _tmp60;
  const Scalar _tmp85 = _tmp84 + position_vector(0, 0) + Scalar(-2.71799795);
  const Scalar _tmp86 = std::pow(Scalar(std::pow(_tmp83, Scalar(2)) + std::pow(_tmp85, Scalar(2))),
                                 Scalar(Scalar(-1) / Scalar(2)));
  const Scalar _tmp87 = _tmp83 * _tmp86;
  const Scalar _tmp88 = _tmp41 * _tmp87 * (_tmp36 * _tmp81 + _tmp78 * _tmp79);
  const Scalar _tmp89 = _tmp85 * _tmp86;
  const Scalar _tmp90 = -_tmp82 * _tmp89 + _tmp84 * _tmp87;
  const Scalar _tmp91 = Scalar(1.0) * _tmp76;
  const Scalar _tmp92 = _tmp36 * _tmp69;
  const Scalar _tmp93 = _tmp41 * _tmp90 * (_tmp67 * _tmp91 - _tmp91 * _tmp92);
  const Scalar _tmp94 = _tmp28 * _tmp58 - _tmp47 * (_tmp28 * _tmp57 + _tmp55) - _tmp54;
  const Scalar _tmp95 = _tmp77 * (_tmp28 * _tmp44 - _tmp42 - _tmp75 * _tmp94) + _tmp94;
  const Scalar _tmp96 = _tmp38 * (-_tmp28 - _tmp80 * _tmp95);
  const Scalar _tmp97 = _tmp41 * _tmp89 * (_tmp36 * _tmp96 + _tmp79 * _tmp95 + Scalar(1.0));
  const Scalar _tmp98 = Scalar(43.164000000000001) - fv1;
  const Scalar _tmp99 = _tmp45 * _tmp47 + _tmp46;
  const Scalar _tmp100 = 0;
  const Scalar _tmp101 = _tmp100 * _tmp98;
  const Scalar _tmp102 = _tmp90 * _tmp91;
  const Scalar _tmp103 = _tmp102 * fh1;
  const Scalar _tmp104 = _tmp89 * _tmp96;
  const Scalar _tmp105 = _tmp81 * _tmp87;
  const Scalar _tmp106 = _tmp74 * _tmp78 * _tmp87;
  const Scalar _tmp107 = _tmp74 * _tmp89 * _tmp95;

  // Output terms (1)
  Eigen::Matrix<Scalar, 4, 1> _res;

  _res(0, 0) = -std::exp(-fh1);
  _res(1, 0) = -(-_tmp88 - _tmp93 - _tmp97) *
               std::exp(_tmp41 * _tmp98 * (_tmp100 * _tmp67 - _tmp100 * _tmp92) + _tmp88 * fh1 +
                        _tmp93 * fh1 + _tmp97 * fh1);
  _res(2, 0) = -(-_tmp102 * _tmp69 + _tmp104 + _tmp105) *
               std::exp(_tmp101 * _tmp69 + _tmp103 * _tmp69 - _tmp104 * fh1 - _tmp105 * fh1);
  _res(3, 0) =
      -(_tmp102 + _tmp106 + _tmp107) * std::exp(-_tmp101 - _tmp103 - _tmp106 * fh1 - _tmp107 * fh1);

  return _res;
}  // NOLINT(readability/fn_size)

// NOLINTNEXTLINE(readability/fn_size)
}  // namespace sym
