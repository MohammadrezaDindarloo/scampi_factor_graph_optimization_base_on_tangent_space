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
 * Symbolic function: IK_residual_func_cost2_wrt_fh1_Nl7
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
Eigen::Matrix<Scalar, 4, 1> IkResidualFuncCost2WrtFh1Nl7(
    const Scalar fh1, const Scalar fv1, const sym::Rot3<Scalar>& DeltaRot,
    const Eigen::Matrix<Scalar, 3, 1>& position_vector, const sym::Rot3<Scalar>& Rot_init,
    const Scalar epsilon) {
  // Total ops: 640

  // Unused inputs
  (void)epsilon;

  // Input arrays
  const Eigen::Matrix<Scalar, 4, 1>& _DeltaRot = DeltaRot.Data();
  const Eigen::Matrix<Scalar, 4, 1>& _Rot_init = Rot_init.Data();

  // Intermediate terms (212)
  const Scalar _tmp0 = Scalar(1.0) / (fh1);
  const Scalar _tmp1 = _tmp0 * fv1;
  const Scalar _tmp2 = std::asinh(_tmp1);
  const Scalar _tmp3 = Scalar(1.0) * _tmp2;
  const Scalar _tmp4 = std::pow(fh1, Scalar(-2));
  const Scalar _tmp5 = _DeltaRot[0] * _Rot_init[2] + _DeltaRot[1] * _Rot_init[3] -
                       _DeltaRot[2] * _Rot_init[0] + _DeltaRot[3] * _Rot_init[1];
  const Scalar _tmp6 = _DeltaRot[0] * _Rot_init[3] - _DeltaRot[1] * _Rot_init[2] +
                       _DeltaRot[2] * _Rot_init[1] + _DeltaRot[3] * _Rot_init[0];
  const Scalar _tmp7 = 2 * _tmp5 * _tmp6;
  const Scalar _tmp8 = -_DeltaRot[0] * _Rot_init[1] + _DeltaRot[1] * _Rot_init[0] +
                       _DeltaRot[2] * _Rot_init[3] + _DeltaRot[3] * _Rot_init[2];
  const Scalar _tmp9 = -2 * _DeltaRot[0] * _Rot_init[0] - 2 * _DeltaRot[1] * _Rot_init[1] -
                       2 * _DeltaRot[2] * _Rot_init[2] + 2 * _DeltaRot[3] * _Rot_init[3];
  const Scalar _tmp10 = _tmp8 * _tmp9;
  const Scalar _tmp11 = Scalar(0.20999999999999999) * _tmp10 + Scalar(0.20999999999999999) * _tmp7;
  const Scalar _tmp12 = -2 * std::pow(_tmp6, Scalar(2));
  const Scalar _tmp13 = -2 * std::pow(_tmp8, Scalar(2));
  const Scalar _tmp14 = Scalar(0.20999999999999999) * _tmp12 +
                        Scalar(0.20999999999999999) * _tmp13 + Scalar(0.20999999999999999);
  const Scalar _tmp15 = 2 * _tmp8;
  const Scalar _tmp16 = _tmp15 * _tmp5;
  const Scalar _tmp17 = _tmp6 * _tmp9;
  const Scalar _tmp18 = _tmp16 - _tmp17;
  const Scalar _tmp19 = -Scalar(0.010999999999999999) * _tmp18;
  const Scalar _tmp20 = -_tmp14 + _tmp19;
  const Scalar _tmp21 = _tmp11 + _tmp20;
  const Scalar _tmp22 = _tmp21 + position_vector(1, 0);
  const Scalar _tmp23 = 1 - 2 * std::pow(_tmp5, Scalar(2));
  const Scalar _tmp24 = Scalar(0.20999999999999999) * _tmp13 + Scalar(0.20999999999999999) * _tmp23;
  const Scalar _tmp25 = _tmp15 * _tmp6;
  const Scalar _tmp26 = _tmp5 * _tmp9;
  const Scalar _tmp27 = _tmp25 + _tmp26;
  const Scalar _tmp28 = -Scalar(0.010999999999999999) * _tmp27;
  const Scalar _tmp29 = -Scalar(0.20999999999999999) * _tmp10 + Scalar(0.20999999999999999) * _tmp7;
  const Scalar _tmp30 = _tmp28 - _tmp29;
  const Scalar _tmp31 = _tmp24 + _tmp30;
  const Scalar _tmp32 = _tmp31 + position_vector(0, 0);
  const Scalar _tmp33 = Scalar(9.6622558468725703) * _tmp2;
  const Scalar _tmp34 =
      -_tmp33 * fh1 -
      Scalar(8.3888750099999996) *
          std::sqrt(
              Scalar(Scalar(0.090254729040973036) *
                         std::pow(Scalar(1 - Scalar(0.39679052492160538) * _tmp32), Scalar(2)) +
                     std::pow(Scalar(-Scalar(0.11920549523123722) * _tmp22 - 1), Scalar(2))));
  const Scalar _tmp35 =
      std::pow(Scalar(_tmp4 * std::pow(fv1, Scalar(2)) + 1), Scalar(Scalar(-1) / Scalar(2)));
  const Scalar _tmp36 = Scalar(0.1034955) * _tmp0;
  const Scalar _tmp37 = _tmp34 * _tmp36;
  const Scalar _tmp38 = -_tmp11;
  const Scalar _tmp39 = _tmp20 + _tmp38;
  const Scalar _tmp40 = _tmp39 + position_vector(1, 0);
  const Scalar _tmp41 = -_tmp24;
  const Scalar _tmp42 = _tmp30 + _tmp41;
  const Scalar _tmp43 = _tmp42 + position_vector(0, 0);
  const Scalar _tmp44 = _tmp22 + Scalar(8.3888750099999996);
  const Scalar _tmp45 = _tmp32 + Scalar(-2.5202214700000001);
  const Scalar _tmp46 = std::pow(Scalar(std::pow(_tmp44, Scalar(2)) + std::pow(_tmp45, Scalar(2))),
                                 Scalar(Scalar(-1) / Scalar(2)));
  const Scalar _tmp47 = _tmp44 * _tmp46;
  const Scalar _tmp48 = Scalar(0.20999999999999999) * _tmp25 - Scalar(0.20999999999999999) * _tmp26;
  const Scalar _tmp49 =
      -Scalar(0.010999999999999999) * _tmp12 - Scalar(0.010999999999999999) * _tmp23;
  const Scalar _tmp50 = Scalar(0.20999999999999999) * _tmp16 + Scalar(0.20999999999999999) * _tmp17;
  const Scalar _tmp51 = _tmp49 - _tmp50;
  const Scalar _tmp52 = _tmp48 + _tmp51;
  const Scalar _tmp53 = _tmp52 * fh1;
  const Scalar _tmp54 = -Scalar(5.1796800000000003) * _tmp18 - _tmp21 * fv1 - _tmp47 * _tmp53;
  const Scalar _tmp55 = Scalar(1.0) * _tmp39;
  const Scalar _tmp56 = -_tmp55;
  const Scalar _tmp57 = _tmp14 + _tmp19;
  const Scalar _tmp58 = _tmp11 + _tmp57;
  const Scalar _tmp59 = _tmp56 + _tmp58;
  const Scalar _tmp60 = _tmp38 + _tmp57;
  const Scalar _tmp61 = Scalar(1.0) / (_tmp56 + _tmp60);
  const Scalar _tmp62 = _tmp28 + _tmp29;
  const Scalar _tmp63 = _tmp41 + _tmp62;
  const Scalar _tmp64 = Scalar(1.0) * _tmp42;
  const Scalar _tmp65 = _tmp61 * (-_tmp63 + _tmp64);
  const Scalar _tmp66 = _tmp59 * _tmp65;
  const Scalar _tmp67 = _tmp24 + _tmp62;
  const Scalar _tmp68 = Scalar(1.0) / (_tmp64 - _tmp66 - _tmp67);
  const Scalar _tmp69 = Scalar(1.0) * _tmp68;
  const Scalar _tmp70 = _tmp66 * _tmp69 + Scalar(1.0);
  const Scalar _tmp71 = Scalar(1.0) * _tmp61;
  const Scalar _tmp72 = _tmp65 * _tmp69;
  const Scalar _tmp73 = -Scalar(1.0) * _tmp70 * _tmp71 + Scalar(1.0) * _tmp72;
  const Scalar _tmp74 = -_tmp48;
  const Scalar _tmp75 = _tmp49 + _tmp50;
  const Scalar _tmp76 = _tmp74 + _tmp75;
  const Scalar _tmp77 = _tmp63 + position_vector(0, 0);
  const Scalar _tmp78 = _tmp77 + Scalar(1.79662371);
  const Scalar _tmp79 = _tmp60 + position_vector(1, 0);
  const Scalar _tmp80 = _tmp79 + Scalar(-4.8333311099999996);
  const Scalar _tmp81 = std::pow(Scalar(std::pow(_tmp78, Scalar(2)) + std::pow(_tmp80, Scalar(2))),
                                 Scalar(Scalar(-1) / Scalar(2)));
  const Scalar _tmp82 = _tmp78 * _tmp81;
  const Scalar _tmp83 = _tmp51 + _tmp74;
  const Scalar _tmp84 = -_tmp76 * _tmp82 + _tmp82 * _tmp83;
  const Scalar _tmp85 = _tmp80 * _tmp81;
  const Scalar _tmp86 = _tmp40 + Scalar(8.3196563700000006);
  const Scalar _tmp87 = _tmp43 + Scalar(1.9874742000000001);
  const Scalar _tmp88 = Scalar(1.0) / (_tmp87);
  const Scalar _tmp89 = _tmp86 * _tmp88;
  const Scalar _tmp90 = Scalar(1.0) / (_tmp82 * _tmp89 - _tmp85);
  const Scalar _tmp91 = _tmp58 + position_vector(1, 0);
  const Scalar _tmp92 = _tmp91 + Scalar(-4.7752063900000001);
  const Scalar _tmp93 = _tmp67 + position_vector(0, 0);
  const Scalar _tmp94 = _tmp93 + Scalar(-2.71799795);
  const Scalar _tmp95 = std::pow(Scalar(std::pow(_tmp92, Scalar(2)) + std::pow(_tmp94, Scalar(2))),
                                 Scalar(Scalar(-1) / Scalar(2)));
  const Scalar _tmp96 = _tmp92 * _tmp95;
  const Scalar _tmp97 = _tmp94 * _tmp95;
  const Scalar _tmp98 = _tmp89 * _tmp97 - _tmp96;
  const Scalar _tmp99 = _tmp90 * _tmp98;
  const Scalar _tmp100 = _tmp48 + _tmp75;
  const Scalar _tmp101 = _tmp83 * _tmp89;
  const Scalar _tmp102 = -_tmp101 * _tmp82 + _tmp76 * _tmp85;
  const Scalar _tmp103 = _tmp100 * _tmp96 - _tmp101 * _tmp97 - _tmp102 * _tmp99;
  const Scalar _tmp104 = -_tmp100 * _tmp97 - _tmp103 * _tmp65 + _tmp83 * _tmp97 - _tmp84 * _tmp99;
  const Scalar _tmp105 =
      std::sqrt(Scalar(std::pow(_tmp86, Scalar(2)) + std::pow(_tmp87, Scalar(2))));
  const Scalar _tmp106 = Scalar(1.0) / (_tmp105);
  const Scalar _tmp107 = _tmp105 * _tmp88;
  const Scalar _tmp108 = _tmp107 * (-_tmp106 * _tmp39 * _tmp87 + _tmp106 * _tmp42 * _tmp86);
  const Scalar _tmp109 = _tmp108 * _tmp82 + _tmp60 * _tmp82 - _tmp63 * _tmp85;
  const Scalar _tmp110 = _tmp108 * _tmp97 - _tmp109 * _tmp99 + _tmp58 * _tmp97 - _tmp67 * _tmp96;
  const Scalar _tmp111 = Scalar(1.0) / (_tmp110);
  const Scalar _tmp112 = _tmp104 * _tmp111;
  const Scalar _tmp113 = _tmp112 * _tmp69;
  const Scalar _tmp114 = Scalar(1.0) * _tmp111;
  const Scalar _tmp115 = -_tmp103 * _tmp114 + _tmp113 * _tmp59;
  const Scalar _tmp116 = _tmp45 * _tmp46;
  const Scalar _tmp117 = -_tmp116 * _tmp21 + _tmp31 * _tmp47;
  const Scalar _tmp118 = Scalar(1.0) * _tmp117;
  const Scalar _tmp119 = _tmp118 * (_tmp113 - _tmp115 * _tmp71);
  const Scalar _tmp120 = _tmp116 * _tmp53 + Scalar(5.1796800000000003) * _tmp27 + _tmp31 * fv1;
  const Scalar _tmp121 = _tmp59 * _tmp61;
  const Scalar _tmp122 = Scalar(1.0) * _tmp121 * _tmp69 - Scalar(1.0) * _tmp69;
  const Scalar _tmp123 = _tmp89 * _tmp90;
  const Scalar _tmp124 = _tmp101 + _tmp102 * _tmp123;
  const Scalar _tmp125 = _tmp123 * _tmp84 - _tmp124 * _tmp65 - _tmp83;
  const Scalar _tmp126 = Scalar(1.0) / (_tmp104);
  const Scalar _tmp127 = _tmp110 * _tmp126;
  const Scalar _tmp128 = _tmp112 * (-_tmp108 + _tmp109 * _tmp123 - _tmp125 * _tmp127);
  const Scalar _tmp129 = _tmp126 * (_tmp125 + _tmp128);
  const Scalar _tmp130 = _tmp59 * _tmp68;
  const Scalar _tmp131 = -_tmp103 * _tmp129 + _tmp124 + _tmp128 * _tmp130;
  const Scalar _tmp132 = Scalar(1.0) * _tmp116 * (_tmp128 * _tmp69 - _tmp131 * _tmp71);
  const Scalar _tmp133 = Scalar(43.164000000000001) - fv1;
  const Scalar _tmp134 = _tmp55 * _tmp65 + _tmp64;
  const Scalar _tmp135 = _tmp134 * _tmp68;
  const Scalar _tmp136 = 0;
  const Scalar _tmp137 = -_tmp103 * _tmp136 - _tmp135 * _tmp59 + _tmp56;
  const Scalar _tmp138 = Scalar(1.0) * _tmp90;
  const Scalar _tmp139 = _tmp102 * _tmp138;
  const Scalar _tmp140 = -_tmp138 * _tmp84 + _tmp139 * _tmp65;
  const Scalar _tmp141 = _tmp112 * (-_tmp109 * _tmp138 - _tmp127 * _tmp140);
  const Scalar _tmp142 = _tmp126 * (_tmp140 + _tmp141);
  const Scalar _tmp143 = -_tmp103 * _tmp142 + _tmp130 * _tmp141 - _tmp139;
  const Scalar _tmp144 = Scalar(1.0) * _tmp47 * (_tmp141 * _tmp69 - _tmp143 * _tmp71);
  const Scalar _tmp145 =
      _tmp119 * fh1 + _tmp120 * _tmp122 + _tmp132 * fh1 +
      Scalar(1.0) * _tmp133 * (-_tmp134 * _tmp69 - _tmp137 * _tmp71 + Scalar(1.0)) + _tmp144 * fh1 +
      _tmp54 * _tmp73;
  const Scalar _tmp146 = _tmp82 * _tmp99;
  const Scalar _tmp147 = _tmp107 * _tmp117 * (-_tmp114 * _tmp146 + _tmp114 * _tmp97);
  const Scalar _tmp148 = -_tmp129 * _tmp98 - _tmp89;
  const Scalar _tmp149 = _tmp82 * _tmp90;
  const Scalar _tmp150 = _tmp107 * _tmp116 * (_tmp129 * _tmp97 + _tmp148 * _tmp149 + Scalar(1.0));
  const Scalar _tmp151 = -_tmp142 * _tmp98 + Scalar(1.0);
  const Scalar _tmp152 = _tmp107 * _tmp47 * (_tmp142 * _tmp97 + _tmp149 * _tmp151);
  const Scalar _tmp153 = -_tmp107 * _tmp133 * (-_tmp136 * _tmp146 + _tmp136 * _tmp97) -
                         _tmp147 * fh1 - _tmp150 * fh1 - _tmp152 * fh1;
  const Scalar _tmp154 = Scalar(1.0) / (_tmp153);
  const Scalar _tmp155 = std::asinh(_tmp145 * _tmp154);
  const Scalar _tmp156 = Scalar(9.6622558468725703) * _tmp153;
  const Scalar _tmp157 =
      -_tmp155 * _tmp156 -
      Scalar(8.3196563700000006) *
          std::sqrt(
              Scalar(std::pow(Scalar(-Scalar(0.12019727204189803) * _tmp40 - 1), Scalar(2)) +
                     Scalar(0.057067943376852184) *
                         std::pow(Scalar(-Scalar(0.50315118556004401) * _tmp43 - 1), Scalar(2))));
  const Scalar _tmp158 = Scalar(0.1034955) * _tmp154;
  const Scalar _tmp159 = _tmp157 * _tmp158;
  const Scalar _tmp160 = std::pow(_tmp153, Scalar(-2));
  const Scalar _tmp161 = -_tmp147 - _tmp150 - _tmp152;
  const Scalar _tmp162 = _tmp160 * _tmp161;
  const Scalar _tmp163 = Scalar(9.6622558468725703) * _tmp161;
  const Scalar _tmp164 = _tmp116 * _tmp52;
  const Scalar _tmp165 = _tmp47 * _tmp52;
  const Scalar _tmp166 = (-_tmp145 * _tmp162 + _tmp154 * (_tmp119 + _tmp122 * _tmp164 + _tmp132 +
                                                          _tmp144 - _tmp165 * _tmp73)) /
                         std::sqrt(Scalar(std::pow(_tmp145, Scalar(2)) * _tmp160 + 1));
  const Scalar _tmp167 = Scalar(1.0) * _tmp155;
  const Scalar _tmp168 = _tmp143 * _tmp47 * _tmp61;
  const Scalar _tmp169 = _tmp120 * _tmp69;
  const Scalar _tmp170 = _tmp61 * _tmp70;
  const Scalar _tmp171 = _tmp115 * _tmp117 * _tmp61;
  const Scalar _tmp172 = _tmp116 * _tmp61;
  const Scalar _tmp173 = _tmp131 * _tmp172;
  const Scalar _tmp174 = -_tmp121 * _tmp169 + _tmp133 * _tmp137 * _tmp61 + _tmp168 * fh1 +
                         _tmp170 * _tmp54 + _tmp171 * fh1 + _tmp173 * fh1;
  const Scalar _tmp175 = _tmp151 * _tmp47 * _tmp90;
  const Scalar _tmp176 = _tmp111 * _tmp118;
  const Scalar _tmp177 = _tmp176 * fh1;
  const Scalar _tmp178 = _tmp116 * _tmp148 * _tmp90;
  const Scalar _tmp179 = _tmp133 * _tmp136;
  const Scalar _tmp180 = _tmp175 * fh1 - _tmp177 * _tmp99 + _tmp178 * fh1 - _tmp179 * _tmp99;
  const Scalar _tmp181 = Scalar(1.0) / (_tmp180);
  const Scalar _tmp182 = std::asinh(_tmp174 * _tmp181);
  const Scalar _tmp183 = Scalar(1.0) * _tmp182;
  const Scalar _tmp184 = Scalar(9.6622558468725703) * _tmp180;
  const Scalar _tmp185 =
      -_tmp182 * _tmp184 -
      Scalar(4.8333311099999996) *
          std::sqrt(
              Scalar(std::pow(Scalar(1 - Scalar(0.20689664689659551) * _tmp79), Scalar(2)) +
                     Scalar(0.13817235445745474) *
                         std::pow(Scalar(-Scalar(0.55659957866191134) * _tmp77 - 1), Scalar(2))));
  const Scalar _tmp186 = Scalar(0.1034955) * _tmp181;
  const Scalar _tmp187 = _tmp185 * _tmp186;
  const Scalar _tmp188 = _tmp175 - _tmp176 * _tmp99 + _tmp178;
  const Scalar _tmp189 = Scalar(9.6622558468725703) * _tmp188;
  const Scalar _tmp190 = std::pow(_tmp180, Scalar(-2));
  const Scalar _tmp191 = _tmp188 * _tmp190;
  const Scalar _tmp192 =
      (-_tmp174 * _tmp191 + _tmp181 * (-_tmp165 * _tmp170 + _tmp168 + _tmp171 -
                                       _tmp172 * _tmp52 * _tmp59 * _tmp69 + _tmp173)) /
      std::sqrt(Scalar(std::pow(_tmp174, Scalar(2)) * _tmp190 + 1));
  const Scalar _tmp193 = _tmp142 * _tmp47;
  const Scalar _tmp194 = _tmp116 * _tmp129;
  const Scalar _tmp195 = _tmp176 + _tmp193 + _tmp194;
  const Scalar _tmp196 = _tmp116 * _tmp128 * _tmp68;
  const Scalar _tmp197 = _tmp113 * _tmp117;
  const Scalar _tmp198 = _tmp141 * _tmp47 * _tmp68;
  const Scalar _tmp199 =
      _tmp133 * _tmp135 + _tmp169 - _tmp196 * fh1 - _tmp197 * fh1 - _tmp198 * fh1 - _tmp54 * _tmp72;
  const Scalar _tmp200 = _tmp177 + _tmp179 + _tmp193 * fh1 + _tmp194 * fh1;
  const Scalar _tmp201 = Scalar(1.0) / (_tmp200);
  const Scalar _tmp202 = std::asinh(_tmp199 * _tmp201);
  const Scalar _tmp203 = Scalar(1.0) * _tmp202;
  const Scalar _tmp204 = Scalar(9.6622558468725703) * _tmp202;
  const Scalar _tmp205 =
      -_tmp200 * _tmp204 -
      Scalar(4.7752063900000001) *
          std::sqrt(
              Scalar(std::pow(Scalar(1 - Scalar(0.20941503221602112) * _tmp91), Scalar(2)) +
                     Scalar(0.32397683292140877) *
                         std::pow(Scalar(1 - Scalar(0.36791786395571047) * _tmp93), Scalar(2))));
  const Scalar _tmp206 = Scalar(0.1034955) * _tmp201;
  const Scalar _tmp207 = _tmp205 * _tmp206;
  const Scalar _tmp208 = Scalar(9.6622558468725703) * _tmp200;
  const Scalar _tmp209 = std::pow(_tmp200, Scalar(-2));
  const Scalar _tmp210 = _tmp195 * _tmp209;
  const Scalar _tmp211 = (-_tmp199 * _tmp210 + _tmp201 * (_tmp164 * _tmp69 + _tmp165 * _tmp72 -
                                                          _tmp196 - _tmp197 - _tmp198)) /
                         std::sqrt(Scalar(std::pow(_tmp199, Scalar(2)) * _tmp209 + 1));

  // Output terms (1)
  Eigen::Matrix<Scalar, 4, 1> _res;

  _res(0, 0) = Scalar(9.6622558468725703) * fh1 *
                   (Scalar(1.0) * _tmp35 * _tmp4 * fv1 * std::cosh(_tmp3) -
                    (-Scalar(0.1034955) * _tmp34 * _tmp4 +
                     _tmp36 * (Scalar(9.6622558468725703) * _tmp1 * _tmp35 - _tmp33)) *
                        std::cosh(_tmp37)) -
               Scalar(9.6622558468725703) * std::sinh(_tmp3) -
               Scalar(9.6622558468725703) * std::sinh(_tmp37);
  _res(1, 0) = _tmp156 * (-Scalar(1.0) * _tmp166 * std::cosh(_tmp167) -
                          (-Scalar(0.1034955) * _tmp157 * _tmp162 +
                           _tmp158 * (-_tmp155 * _tmp163 - _tmp156 * _tmp166)) *
                              std::cosh(_tmp159)) +
               _tmp163 * (-std::sinh(_tmp159) - std::sinh(_tmp167));
  _res(2, 0) = _tmp184 * (-Scalar(1.0) * _tmp192 * std::cosh(_tmp183) -
                          (-Scalar(0.1034955) * _tmp185 * _tmp191 +
                           _tmp186 * (-_tmp182 * _tmp189 - _tmp184 * _tmp192)) *
                              std::cosh(_tmp187)) +
               _tmp189 * (-std::sinh(_tmp183) - std::sinh(_tmp187));
  _res(3, 0) = Scalar(9.6622558468725703) * _tmp195 * (-std::sinh(_tmp203) - std::sinh(_tmp207)) +
               _tmp208 * (-Scalar(1.0) * _tmp211 * std::cosh(_tmp203) -
                          (-Scalar(0.1034955) * _tmp205 * _tmp210 +
                           _tmp206 * (-_tmp195 * _tmp204 - _tmp208 * _tmp211)) *
                              std::cosh(_tmp207));

  return _res;
}  // NOLINT(readability/fn_size)

// NOLINTNEXTLINE(readability/fn_size)
}  // namespace sym