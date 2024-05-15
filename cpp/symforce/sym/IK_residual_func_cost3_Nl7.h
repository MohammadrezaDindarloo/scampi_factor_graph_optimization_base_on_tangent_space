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
 * Symbolic function: IK_residual_func_cost3_Nl7
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
Eigen::Matrix<Scalar, 4, 1> IkResidualFuncCost3Nl7(
    const Scalar fh1, const Scalar fv1, const sym::Rot3<Scalar>& DeltaRot,
    const Eigen::Matrix<Scalar, 3, 1>& position_vector, const sym::Rot3<Scalar>& Rot_init,
    const Scalar epsilon) {
  // Total ops: 280

  // Unused inputs
  (void)epsilon;

  // Input arrays
  const Eigen::Matrix<Scalar, 4, 1>& _DeltaRot = DeltaRot.Data();
  const Eigen::Matrix<Scalar, 4, 1>& _Rot_init = Rot_init.Data();

  // Intermediate terms (106)
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
  const Scalar _tmp15 = _tmp11 - _tmp14;
  const Scalar _tmp16 = _tmp15 + _tmp7;
  const Scalar _tmp17 = Scalar(1.0) * _tmp16;
  const Scalar _tmp18 = _tmp11 + _tmp14;
  const Scalar _tmp19 = _tmp18 + _tmp7;
  const Scalar _tmp20 = Scalar(0.20999999999999999) * _tmp2 - Scalar(0.20999999999999999) * _tmp5;
  const Scalar _tmp21 = _tmp1 * _tmp8;
  const Scalar _tmp22 = _tmp0 * _tmp4;
  const Scalar _tmp23 =
      -Scalar(0.010999999999999999) * _tmp21 - Scalar(0.010999999999999999) * _tmp22;
  const Scalar _tmp24 = 1 - 2 * std::pow(_tmp0, Scalar(2));
  const Scalar _tmp25 = Scalar(0.20999999999999999) * _tmp13 + Scalar(0.20999999999999999) * _tmp24;
  const Scalar _tmp26 = _tmp23 - _tmp25;
  const Scalar _tmp27 = _tmp20 + _tmp26;
  const Scalar _tmp28 = -_tmp20;
  const Scalar _tmp29 = _tmp26 + _tmp28;
  const Scalar _tmp30 = Scalar(1.0) * _tmp29;
  const Scalar _tmp31 = (-_tmp27 + _tmp30) / (-_tmp17 + _tmp19);
  const Scalar _tmp32 = position_vector(1, 0) + Scalar(-110.0);
  const Scalar _tmp33 = _tmp19 + _tmp32;
  const Scalar _tmp34 = position_vector(0, 0) + Scalar(125.0);
  const Scalar _tmp35 = _tmp27 + _tmp34;
  const Scalar _tmp36 = std::pow(Scalar(std::pow(_tmp33, Scalar(2)) + std::pow(_tmp35, Scalar(2))),
                                 Scalar(Scalar(-1) / Scalar(2)));
  const Scalar _tmp37 = _tmp33 * _tmp36;
  const Scalar _tmp38 = position_vector(1, 0) + Scalar(110.0);
  const Scalar _tmp39 = _tmp16 + _tmp38;
  const Scalar _tmp40 = _tmp29 + _tmp34;
  const Scalar _tmp41 = Scalar(1.0) / (_tmp40);
  const Scalar _tmp42 = _tmp39 * _tmp41;
  const Scalar _tmp43 = _tmp35 * _tmp36;
  const Scalar _tmp44 = Scalar(1.0) / (-_tmp37 + _tmp42 * _tmp43);
  const Scalar _tmp45 = Scalar(0.20999999999999999) * _tmp10 + Scalar(0.20999999999999999) * _tmp9;
  const Scalar _tmp46 =
      -Scalar(0.010999999999999999) * _tmp12 - Scalar(0.010999999999999999) * _tmp24;
  const Scalar _tmp47 = Scalar(0.20999999999999999) * _tmp21 - Scalar(0.20999999999999999) * _tmp22;
  const Scalar _tmp48 = _tmp46 - _tmp47;
  const Scalar _tmp49 = -_tmp45 + _tmp48;
  const Scalar _tmp50 = _tmp43 * _tmp49;
  const Scalar _tmp51 = _tmp45 + _tmp48;
  const Scalar _tmp52 = _tmp37 * _tmp51 - _tmp42 * _tmp50;
  const Scalar _tmp53 = _tmp44 * _tmp52;
  const Scalar _tmp54 = -_tmp43 * _tmp51 + _tmp50;
  const Scalar _tmp55 = _tmp44 * _tmp54;
  const Scalar _tmp56 = Scalar(1.0) * _tmp31 * _tmp53 - Scalar(1.0) * _tmp55;
  const Scalar _tmp57 = _tmp18 + _tmp6;
  const Scalar _tmp58 = _tmp32 + _tmp57;
  const Scalar _tmp59 = _tmp23 + _tmp25;
  const Scalar _tmp60 = _tmp20 + _tmp59;
  const Scalar _tmp61 = position_vector(0, 0) + Scalar(-125.0);
  const Scalar _tmp62 = _tmp60 + _tmp61;
  const Scalar _tmp63 = std::pow(Scalar(std::pow(_tmp58, Scalar(2)) + std::pow(_tmp62, Scalar(2))),
                                 Scalar(Scalar(-1) / Scalar(2)));
  const Scalar _tmp64 = _tmp62 * _tmp63;
  const Scalar _tmp65 = _tmp45 + _tmp46 + _tmp47;
  const Scalar _tmp66 = _tmp58 * _tmp63;
  const Scalar _tmp67 = _tmp42 * _tmp64 - _tmp66;
  const Scalar _tmp68 = _tmp44 * _tmp67;
  const Scalar _tmp69 = _tmp42 * _tmp49;
  const Scalar _tmp70 = -_tmp31 * (-_tmp52 * _tmp68 - _tmp64 * _tmp69 + _tmp65 * _tmp66) +
                        _tmp49 * _tmp64 - _tmp54 * _tmp68 - _tmp64 * _tmp65;
  const Scalar _tmp71 = Scalar(1.0) / (_tmp70);
  const Scalar _tmp72 =
      std::sqrt(Scalar(std::pow(_tmp39, Scalar(2)) + std::pow(_tmp40, Scalar(2))));
  const Scalar _tmp73 = Scalar(1.0) / (_tmp72);
  const Scalar _tmp74 = _tmp41 * _tmp72;
  const Scalar _tmp75 = _tmp74 * (-_tmp16 * _tmp40 * _tmp73 + _tmp29 * _tmp39 * _tmp73);
  const Scalar _tmp76 = _tmp44 * (_tmp19 * _tmp43 - _tmp27 * _tmp37 + _tmp43 * _tmp75);
  const Scalar _tmp77 = _tmp57 * _tmp64 - _tmp60 * _tmp66 + _tmp64 * _tmp75 - _tmp67 * _tmp76;
  const Scalar _tmp78 = _tmp71 * _tmp77;
  const Scalar _tmp79 = Scalar(1.0) / (_tmp77);
  const Scalar _tmp80 = _tmp70 * _tmp79;
  const Scalar _tmp81 = _tmp56 + _tmp80 * (-_tmp56 * _tmp78 - Scalar(1.0) * _tmp76);
  const Scalar _tmp82 = _tmp64 * _tmp71;
  const Scalar _tmp83 = _tmp67 * _tmp71;
  const Scalar _tmp84 = -_tmp81 * _tmp83 + Scalar(1.0);
  const Scalar _tmp85 = _tmp43 * _tmp44;
  const Scalar _tmp86 = _tmp15 + _tmp6;
  const Scalar _tmp87 = _tmp38 + _tmp86;
  const Scalar _tmp88 = _tmp28 + _tmp59;
  const Scalar _tmp89 = _tmp61 + _tmp88;
  const Scalar _tmp90 = std::pow(Scalar(std::pow(_tmp87, Scalar(2)) + std::pow(_tmp89, Scalar(2))),
                                 Scalar(Scalar(-1) / Scalar(2)));
  const Scalar _tmp91 = _tmp87 * _tmp90;
  const Scalar _tmp92 = _tmp91 * fh1;
  const Scalar _tmp93 = Scalar(1.0) * _tmp79;
  const Scalar _tmp94 = _tmp43 * _tmp68;
  const Scalar _tmp95 = _tmp89 * _tmp90;
  const Scalar _tmp96 = fh1 * (-_tmp86 * _tmp95 + _tmp88 * _tmp91);
  const Scalar _tmp97 = -_tmp31 * (_tmp42 * _tmp53 + _tmp69) + _tmp42 * _tmp55 - _tmp49;
  const Scalar _tmp98 = _tmp80 * (_tmp42 * _tmp76 - _tmp75 - _tmp78 * _tmp97) + _tmp97;
  const Scalar _tmp99 = -_tmp42 - _tmp83 * _tmp98;
  const Scalar _tmp100 = _tmp95 * fh1;
  const Scalar _tmp101 = Scalar(333.54000000000002) - fv1;
  const Scalar _tmp102 = _tmp17 * _tmp31 + _tmp30;
  const Scalar _tmp103 = 0;
  const Scalar _tmp104 = _tmp101 * _tmp103;
  const Scalar _tmp105 = _tmp93 * _tmp96;

  // Output terms (1)
  Eigen::Matrix<Scalar, 4, 1> _res;

  _res(0, 0) = std::exp(-fh1);
  _res(1, 0) = std::exp(_tmp100 * _tmp74 * (_tmp82 * _tmp98 + _tmp85 * _tmp99 + Scalar(1.0)) +
                        _tmp101 * _tmp74 * (_tmp103 * _tmp64 - _tmp103 * _tmp94) +
                        _tmp74 * _tmp92 * (_tmp81 * _tmp82 + _tmp84 * _tmp85) +
                        _tmp74 * _tmp96 * (_tmp64 * _tmp93 - _tmp93 * _tmp94));
  _res(2, 0) = std::exp(-_tmp100 * _tmp44 * _tmp99 + _tmp104 * _tmp68 + _tmp105 * _tmp68 -
                        _tmp44 * _tmp84 * _tmp92);
  _res(3, 0) = std::exp(-_tmp100 * _tmp71 * _tmp98 - _tmp104 - _tmp105 - _tmp71 * _tmp81 * _tmp92);

  return _res;
}  // NOLINT(readability/fn_size)

// NOLINTNEXTLINE(readability/fn_size)
}  // namespace sym
