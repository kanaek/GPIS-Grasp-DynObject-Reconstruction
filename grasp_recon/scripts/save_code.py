def info_gain_cost_gradient(u, dyn_model):
        u = u['u']
        cloud_center = dyn_model.FK_pointcloud_center(u)
        temp_u = np.copy(u)
        temp_u[:] = 0.0
        # plus_difference = u+dyn_model.finite_array
        wm = dyn_model.wm
        cloud_out_1 = dyn_model.FK_pointcloud(u)
        after_ray, temp_list, distance = dyn_model.call_ray_cast(cloud_out_1, cloud_center, return_index=True)
        if distance < -0.01:
                distance = 0
        elif distance >= -0.01:
                distance = distance
        test_jacobian = dyn_model.chain.jacobian_pointcloud(u, pos=None, base_link='base', end_link='cloud_center')
        fake_distance = np.array([distance, distance, distance])
        gradient_sdf = fake_distance * test_jacobian[0:3, :]
        gradient_sdf = np.array(gradient_sdf)[0, :]
        # print('gradient_sdf', gradient_sdf)

        if after_ray.shape[0] == 0:
                print('out of scope after')
                return np.array([[0., 0., 0., 0., 0., 0., 0.]]) - lbr4.cone * gradient_sdf
        # var = dyn_model.gpy.predict(after_ray, full_cov=True)
        # var = var[1]
        # temp_var = np.dot(2.0 * np.pi * np.exp(1), var)
        # info_gain = np.linalg.det(temp_var)
        # info_gain_before = np.log(info_gain) * (-0.5)

        # var = dyn_model.gpflow.predict_f_full_cov(after_ray)
        # var = var[1]
        # temp_var = np.zeros((var.shape[0], var.shape[1]), dtype=np.float)
        # for i in range(var.shape[0]):
        #     for j in range(var.shape[1]):
        #         temp_var[i][j] = var[i][j][0]
        #         if i == j:
        #             temp_var[i][j] += 0.1
        # temp_var = np.dot(2.0 * np.pi * np.exp(1), temp_var)
        # info_gain = np.linalg.det(temp_var)
        # info_gain_before = np.log(info_gain) * (-0.5)

        for q in range(0, len(u)):
                # q = 2
                plus_difference = np.copy(u)
                plus_difference[q] = plus_difference[q] + dyn_model.finite_difference[q]
                cloud_out_1 = dyn_model.FK_pointcloud(plus_difference)
                after_ray, temp_list, distance = dyn_model.call_ray_cast(cloud_out_1, cloud_center, return_index=True)
                if after_ray.shape[0] == 0:
                        print('out of scope in finite difference')
                        temp_u[q] = 0
                        continue
                        # break
                        # return np.array([[0., 0., 0., 0., 0., 0., 0.]])
                var = dyn_model.gpy.predict(after_ray, full_cov=True)
                var = var[1]
                temp_var = np.dot(2.0 * np.pi * np.exp(1), var)
                info_gain = np.linalg.det(temp_var)
                info_gain_after = np.log(info_gain) * (-0.5)
                # temp_var = np.zeros((var.shape[0], var.shape[1]), dtype=np.float)
                # for i in range(var.shape[0]):
                #     for j in range(var.shape[1]):
                #         temp_var[i][j] = var[i][j][0]
                #         if i == j:
                #             temp_var[i][j] += 0.1
                # temp_var = np.dot(2.0 * np.pi * np.exp(1), temp_var)
                # info_gain = np.linalg.det(temp_var)
                # info_gain_after = np.log(info_gain) * (-0.5)

                plus_difference = np.copy(u)
                plus_difference[q] = plus_difference[q] - dyn_model.finite_difference[q]
                cloud_out_1 = dyn_model.FK_pointcloud(plus_difference)
                after_ray, temp_list, distance = dyn_model.call_ray_cast(cloud_out_1, cloud_center, return_index=True)
                if after_ray.shape[0] == 0:
                        print('out of scope in finite difference')
                        temp_u[q] = 0
                        break
                var = dyn_model.gpy.predict(after_ray, full_cov=True)
                var = var[1]
                temp_var = np.dot(2.0 * np.pi * np.exp(1), var)
                info_gain = np.linalg.det(temp_var)
                info_gain_before = np.log(info_gain) * (-0.5)
                # var = dyn_model.gpflow.predict_f_full_cov(after_ray)
                # var = var[1]
                # temp_var = np.zeros((var.shape[0], var.shape[1]), dtype=np.float)
                # for i in range(var.shape[0]):
                #     for j in range(var.shape[1]):
                #         temp_var[i][j] = var[i][j][0]
                #         if i == j:
                #             temp_var[i][j] += 0.1
                # temp_var = np.dot(2.0 * np.pi * np.exp(1), temp_var)
                # info_gain = np.linalg.det(temp_var)
                # info_gain_before = np.log(info_gain) * (-0.5)


                info_gain_difference = info_gain_after - info_gain_before
                degree = rad_to_degree(dyn_model.finite_difference[q])
                # print('degree',degree)
                # t = (info_gain_difference / dyn_model.finite_difference) * (-wm)
                t = (info_gain_difference / (2 * degree)) * (-wm) * 0.3
                print('differe', q, t)
                temp_u[q] = t

                # break #for testing which direction it can go down

        return [temp_u - lbr4.cone * gradient_sdf]

        # inv_matrix = np.linalg.inv(temp_var)
        # transpose_matrix = np.transpose(inv_matrix)
        # grad = dyn_model.gpy.predictive_gradients(after_ray)
        # grad = grad[1]
        # temp_grad = np.dot(transpose_matrix, grad)
        # final_grad = []
        # for i in range(len(temp_list)):
        #     # virtual_link = str(temp_list[i])
        #     # test_jacobian = dyn_model.chain.jacobian_pointcloud(u, pos=None, base_link='base',
        #     #                                                     end_link=virtual_link)
        #     test_jacobian_point = test_jacobian[0:3, :]
        #     final = np.dot([temp_grad[i]], test_jacobian_point)
        #     final_grad.append(final)
        # final_grad = np.dot(np.mean(final_grad, axis=0), 0.5)
        # final_grad = np.dot(final_grad, wm*0.1)
        # print('final_grad', final_grad)
        # return final_grad


    # var = m2.predict_f_full_cov(after_ray)
    # var = var[1]
    # print('gpu finished',var)
    # temp_var = np.zeros((var.shape[0], var.shape[1]), dtype=np.float)
    # for i in range(var.shape[0]):
    #     for j in range(var.shape[1]):
    #         temp_var[i][j] = var[i][j][0]
    #         if i == j:
    #             temp_var[i][j] += 0.1
    # temp_var = np.dot(2.0 * np.pi * np.exp(1), temp_var)
    # info_gain = np.linalg.det(temp_var)
    # info_gain = np.log(info_gain) * 0.5
    # print('real_condtion_entropy',info_gain)


# cloud_pub = rospy.Publisher('gpis_debug',PointCloud2,queue_size=10)
# rate = rospy.Rate(10)
# print('before debug')
# while not rospy.is_shutdown():
#     cloud_out_ = []
#     current_angles = [limb_interface.joint_angle(joint) for joint in limb_interface.joint_names()]
#     for i in range(len(cloud_out)):
#         virtual_link = str(i)
#         test_link = chain.forward_pointcloud(current_angles, end_link=virtual_link, base_link='base')
#         pose = np.zeros(3)
#         pose[0:3] = test_link[0:3, 3].ravel()
#         # print(pose)
#         cloud_out_.append(pose)
#
#     cloud_out_1 = transform_cloud(cloud_out_, transform)
#
#     cloud_out_center = []
#     test_link = chain.forward_pointcloud(current_angles, end_link='cloud_center', base_link='base')
#     pose = np.zeros(3)
#     pose[0:3] = test_link[0:3, 3].ravel()
#     cloud_out_center.append(pose)
#     cloud_out_center = transform_cloud(cloud_out_center, transform)
#     point_center = Point()
#     point_center.x = cloud_out_center[0][0]
#     point_center.y = cloud_out_center[0][1]
#     point_center.z = cloud_out_center[0][2]
#
#     HEADER = Header(frame_id='/camera_link')
#     fields = [PointField('x', 0, PointField.FLOAT32, 1),
#               PointField('y', 4, PointField.FLOAT32, 1),
#               PointField('z', 8, PointField.FLOAT32, 1)
#               ]
#     obj_cloud_gpis = pc2.create_cloud(HEADER, fields, cloud_out_1)
#
#     rospy.wait_for_service('raycast')
#     raycast_srv = rospy.ServiceProxy('raycast', raycast)
#     raycast_resp = raycast_srv(obj_cloud_gpis,point_center)
#     temp_list = raycast_resp.array
#     distance = raycast_resp.distance
#     after_ray = []
#     for i in range(len(temp_list)):
#         after_ray.append(list(cloud_out_[temp_list[i]][0:3]))
#
#     after_ray = np.array(after_ray)
#     HEADER = Header(frame_id='/base')
#     fields = [PointField('x', 0, PointField.FLOAT32, 1),
#               PointField('y', 4, PointField.FLOAT32, 1),
#               PointField('z', 8, PointField.FLOAT32, 1)
#               ]
#     after_gpis = pc2.create_cloud(HEADER, fields, after_ray)
#     cloud_pub.publish(after_gpis)
#     print('distance', distance)
#     rate.sleep()
# print('after_ray')

('g_con', [-0.28044140734991918, 0.33908339181863612, -0.15747697148268713, -0.16177437572079456, -0.13088482585846195, -0.20184609312621538, 0.12883153311465201, 2.5896435946637375e-05, -1.125353586745037e-05, 7.9641960992749006e-06, -2.6433083178067918e-05, -5.3085766575922122e-06, -1.133256465735144e-05, 0.0])
('g_con_grad', array([[ 1.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,
         0.],
       [ 0.,  1.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,
         0.],
       [ 0.,  0.,  1.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,
         0.],
       [ 0.,  0.,  0.,  1.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,
         0.],
       [ 0.,  0.,  0.,  0.,  1.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,
         0.],
       [ 0.,  0.,  0.,  0.,  0.,  1.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,
         0.],
       [ 0.,  0.,  0.,  0.,  0.,  0.,  1.,  0.,  0.,  0.,  0.,  0.,  0.,
         0.],
       [-1.,  0.,  0.,  0.,  0.,  0.,  0.,  1.,  0.,  0.,  0.,  0.,  0.,
         0.],
       [ 0., -1.,  0.,  0.,  0.,  0.,  0.,  0.,  1.,  0.,  0.,  0.,  0.,
         0.],
       [ 0.,  0., -1.,  0.,  0.,  0.,  0.,  0.,  0.,  1.,  0.,  0.,  0.,
         0.],
       [ 0.,  0.,  0., -1.,  0.,  0.,  0.,  0.,  0.,  0.,  1.,  0.,  0.,
         0.],
       [ 0.,  0.,  0.,  0., -1.,  0.,  0.,  0.,  0.,  0.,  0.,  1.,  0.,
         0.],
       [ 0.,  0.,  0.,  0.,  0., -1.,  0.,  0.,  0.,  0.,  0.,  0.,  1.,
         0.],
       [ 0.,  0.,  0.,  0.,  0.,  0., -1.,  0.,  0.,  0.,  0.,  0.,  0.,
         1.]]))

('ubound0', array([-1.70167994, -2.147     , -3.05417994, -0.05      , -3.059     ,
       -1.57079633, -3.059     ]))
('ubound1', array([ 1.70167994,  1.047     ,  3.05417994,  2.618     ,  3.059     ,
        2.094     ,  3.059     ]))

when j0: (ufinal,[-1.01935909, -0.93444153,  0.7592138 ,  1.45420466, -0.45813642,
        0.82359786,  0.05361328])

[[ -2.18027385e-01  -7.35046878e-01  -8.02988561e-04   1.39925382e+00
   -1.05842596e-01   8.02229116e-01  -1.53441008e+00]
 [ -2.52215465e-01  -7.36913606e-01  -8.67785263e-04   1.43079096e+00
   -1.07909730e-01   7.70398609e-01  -1.56762651e+00]
 [ -2.69309505e-01  -7.37846970e-01  -9.00183613e-04   1.44655953e+00
   -1.08943297e-01   7.54483355e-01  -1.58423472e+00]
 [ -2.86403545e-01  -7.38780334e-01  -9.32581964e-04   1.46232809e+00
   -1.09976864e-01   7.38568102e-01  -1.60084293e+00]
 [ -3.03497585e-01  -7.39713698e-01  -9.64980315e-04   1.47809666e+00
   -1.11010431e-01   7.22652848e-01  -1.61745114e+00]
 [ -3.20591625e-01  -7.40647062e-01  -9.97378665e-04   1.49386523e+00
   -1.12043998e-01   7.06737595e-01  -1.63405935e+00]
 [ -3.37685665e-01  -7.41580425e-01  -1.02977702e-03   1.50963379e+00
   -1.13077565e-01   6.90822341e-01  -1.65066757e+00]
 [ -3.54779705e-01  -7.42513789e-01  -1.06217537e-03   1.52540236e+00
   -1.14111132e-01   6.74907088e-01  -1.66727578e+00]
 [ -3.71873745e-01  -7.43447153e-01  -1.09457372e-03   1.54117093e+00
   -1.15144699e-01   6.58991834e-01  -1.68388399e+00]
 [ -3.88967785e-01  -7.44380517e-01  -1.12697207e-03   1.55693949e+00
   -1.16178266e-01   6.43076581e-01  -1.70049220e+00]
 [ -4.24748713e-01  -7.28465010e-01   5.85098908e-04   1.56819283e+00
   -1.18137810e-01   6.13811673e-01  -1.73310299e+00]
 [ -4.58385110e-01  -6.98137043e-01   3.72514370e-03   1.56006542e+00
   -1.19804570e-01   5.89782296e-01  -1.76190762e+00]
 [ -4.75203309e-01  -6.82973059e-01   5.29516609e-03   1.55600172e+00
   -1.20637950e-01   5.77767607e-01  -1.77630994e+00]
 [ -4.92021508e-01  -6.67809075e-01   6.86518848e-03   1.55193801e+00
   -1.21471330e-01   5.65752918e-01  -1.79071226e+00]
 [ -5.08839707e-01  -6.52645092e-01   8.43521088e-03   1.54787430e+00
   -1.22304710e-01   5.53738230e-01  -1.80511457e+00]
 [ -5.25657905e-01  -6.37481108e-01   1.00052333e-02   1.54381060e+00
   -1.23138090e-01   5.41723541e-01  -1.81951689e+00]
 [ -5.42476104e-01  -6.22317124e-01   1.15752557e-02   1.53974689e+00
   -1.23971469e-01   5.29708852e-01  -1.83391921e+00]
 [ -5.59294303e-01  -6.07153141e-01   1.31452781e-02   1.53568319e+00
   -1.24804849e-01   5.17694164e-01  -1.84832152e+00]
 [ -5.76112502e-01  -5.91989157e-01   1.47153005e-02   1.53161948e+00
   -1.25638229e-01   5.05679475e-01  -1.86272384e+00]
 [ -5.92930701e-01  -5.76825173e-01   1.62853228e-02   1.52755578e+00
   -1.26471609e-01   4.93664787e-01  -1.87712616e+00]
 [ -6.09748899e-01  -5.61661189e-01   1.78553452e-02   1.52349207e+00
   -1.27304989e-01   4.81650098e-01  -1.89152848e+00]]
