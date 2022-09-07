import numpy as np

import matplotlib.pyplot as plt

data = np.array([
[-0.0287807, -8.38288, 36.8131, 0.336],                                                                                                                                                                                                     
[-0.0370175, -7.97213, 744.59, 0.337],                                                                                                                                                                                                      
[-0.0445382, -7.0078, 1110.07, 0.338],                                                                                                                                                                                                                                     
[-0.0473417, -6.54439, 1113.28, 0.338414],                                                                                                                                                                                                  
[-0.0532928, -5.61279, 660.218, 0.339399],                                                                                                                                                                                                  
[-0.0553819, -5.27025, 1110.52, 0.340383],                                                                                                                                                                                                  
[-0.0599448, -3.76062, 1808.46, 0.341381],                                                                                                                                                                                                  
[-0.0627891, -1.81402, 1935.97, 0.342397],                                                                                                                                                                                                  
[-0.0636694, 0.0516038, 1624.93, 0.343428],                                                                                                                                                                                                 
[-0.0629011, 1.40025, 1058.29, 0.344421],                                                                                                                                                                                                   
[-0.0610864, 2.11957, 375.672, 0.34542],                                                                                                                                                                                                    
[-0.0589164, 2.16547, -264.817, 0.346409],                                                                                                                                                                                                  
[-0.0568431, 1.58844, -752.934, 0.347488],                                                                                                                                                                                                  
[-0.0557207, 0.813676, -856.546, 0.348416],                                                                                                                                                                                                 
[-0.0552981, 0.0897494, -494.308, 0.349418],                                                                                                                                                                                                
[-0.0552986, -0.0808124, -330.606, 0.350416],                                                                                                                                                                                               
[-0.0555163, -0.257875, -18.9856, 0.351516],                                                                                                                                                                                                
[-0.0557399, -0.200152, 127.064, 0.352446],                                                                                                                                                                                                 
[-0.0558625, -0.03491, 189.389, 0.353446],                                                                                                                                                                                                  
[-0.0557935, 0.163819, 177.197, 0.354499],                                                                                                                                                                                                  
[-0.0555649, 0.308531, 122.5, 0.355449],                                                                                                                                                                                                    
[-0.055193, 0.39484, 42.7613, 0.356486],                                                                                                                                                                                                    
[-0.0548253, 0.403053, -22.1271, 0.357397],                                                                                                                                                                                                 
[-0.0544403, 0.356373, -63.4386, 0.358401],                                                                                                                                                                                                 
[-0.054114, 0.294736, -47.2295, 0.359408],                                                                                                                                                                                                  
[-0.0540023, 0.273277, -61.2077, 0.360393],                                                                                                                                                                                                 
[-0.0537565, 0.199783, -76.1193, 0.361426],                                                                                                                                                                                                 
[-0.0535922, 0.125908, -66.2325, 0.36244],                                                                                                                                                                                                  
[-0.053497, 0.0723495, -40.8865, 0.363422],                                                                                                                                                                                                 
[-0.0534397, 0.048271, -6.61499, 0.364418],                                                                                                                                                                                                 
[-0.0533832, 0.0624557, 32.0842, 0.365509],                                                                                                                                                                                                 
[-0.0532977, 0.110705, 61.0369, 0.366525],                                                                                                                                                                                                  
[-0.053175, 0.171074, 74.3508, 0.367402],                                                                                                                                                                                                   
[-0.0529635, 0.245962, 69.3723, 0.368415],                                                                                                                                                                                                  
[-0.052681, 0.301963, 34.8093, 0.369435],                                                                                                                                                                                                   
[-0.0522386, 0.254908, -75.8989, 0.371513],                                                                                                                                                                                                 
[-0.0520424, 0.177905, -89.2855, 0.372416],                                                                                                                                                                                                 
[-0.0519091, 0.0962814, -72.3785, 0.373398],                                                                                                                                                                                                
[-0.0518412, 0.0404992, -31.2851, 0.374446],                                                                                                                                                                                                
[-0.0518104, 0.0322308, 14.0659, 0.375384],                                                                                                                                                                                                 
[-0.0517636, 0.0693991, 59.0427, 0.37638],                                                                                                                                                                                                  
[-0.0516578, 0.145651, 88.7315, 0.377386],                                                                                                                                                                                                  
[-0.0514654, 0.238147, 89.7955, 0.37839],                                                                                                                                                                                                   
[-0.0511798, 0.313037, 48.3565, 0.379413],
[-0.0510564, 0.322538, 2.48854, 0.380387],                                                                                                                                                                                                  
[-0.0507624, 0.286406, -69.8327, 0.381335],                                                                                                                                                                                                 
[-0.0505126, 0.198148, -94.8862, 0.382357],                                                                                                                                                                                                 
[-0.0503595, 0.10707, -79.6351, 0.383369],                                                                                                                                                                                                  
[-0.0502863, 0.0460757, -39.4461, 0.384368],                                                                                                                                                                                                
[-0.0502515, 0.0320104, 12.1042, 0.38537],                                                                                                                                                                                                  
[-0.0502044, 0.069827, 61.3207, 0.386378],                                                                                                                                                                                                  
[-0.0500989, 0.148453, 93.5186, 0.387369],                                                                                                                                                                                                  
[-0.0498918, 0.250146, 95.3172, 0.38841],                                                                                                                                                                                                   
[-0.0496014, 0.3277, 52.4806, 0.389402],                                                                                                                                                                                                    
[-0.0494758, 0.338211, 5.23513, 0.390375],                                                                                                                                                                                                  
[-0.0491423, 0.29602, -76.2385, 0.391405],                                                                                                                                                                                                  
[-0.0488882, 0.203056, -99.6763, 0.392415],                                                                                                                                                                                                 
[-0.0487388, 0.113238, -83.9688, 0.393367],                                                                                                                                                                                                 
[-0.0486609, 0.0482861, -41.4534, 0.394377],                                                                                                                                                                                                
[-0.0486252, 0.0332053, 11.4123, 0.395356],                                                                                                                                                                                                 
[-0.0485773, 0.071222, 62.9506, 0.396357],                                                                                                                                                                                                  
[-0.0484582, 0.159528, 98.82, 0.397419],                                                                                                                                                                                                    
[-0.0482495, 0.261331, 99.8366, 0.398411],                                                                                                                                                                                                  
[-0.0479709, 0.338339, 60.1034, 0.399331],                                                                                                                                                                                                  
[-0.0478386, 0.351205, 9.11534, 0.400382],                                                                                                                                                                                                  
[-0.0474969, 0.311629, -76.4141, 0.401391],                                                                                                                                                                                                 
[-0.0472179, 0.211947, -103.005, 0.402448],                                                                                                                                                                                                 
[-0.0470622, 0.119861, -86.6307, 0.403393],                                                                                                                                                                                                 
[-0.0469791, 0.0533964, -42.6852, 0.404395],                                                                                                                                                                                                
[-0.0469354, 0.039425, 17.262, 0.405461],                                                                                                                                                                                                   
[-0.0468832, 0.0797075, 67.0194, 0.406399],                                                                                                                                                                                                 
[-0.0467599, 0.168404, 102.621, 0.407418],                                                                                                                                                                                                  
[-0.0465482, 0.271266, 104.499, 0.408381],                                                                                                                                                                                                  
[-0.0462197, 0.359291, 56.788, 0.40941],                                                                                                                                                                                                    
[-0.0460586, 0.370657, -2.45843, 0.410439],                                                                                                                                                                                                 
[-0.0456971, 0.318672, -86.9766, 0.411466],                                                                                                                                                                                                 
[-0.0454459, 0.225152, -109.449, 0.412384],                                                                                                                                                                                                 
[-0.0452669, 0.116032, -89.663, 0.413445],                                                                                                                                                                                                  
[-0.0451875, 0.0474746, -40.929, 0.414468],                                                                                                                                                                                                 
[-0.0451537, 0.0346223, 13.5463, 0.415387],                                                                                                                                                                                                 
[-0.0450991, 0.080089, 71.5908, 0.41643],                                                                                                                                                                                                   
[-0.0449831, 0.167505, 106.639, 0.417388],                                                                                                                                                                                                  
[-0.0447389, 0.287014, 107.394, 0.418463],                                                                                                                                                                                                  
[-0.044389, 0.375351, 51.4189, 0.419505],                                                                                                                                                                                                   
[-0.0442355, 0.384493, -3.7746, 0.420402],                                                                                                                                                                                                  
[-0.0438549, 0.328025, -91.6955, 0.421448],                                                                                                                                                                                                 
[-0.0435767, 0.219873, -113.969, 0.422456],                                                                                                                                                                                                 
[-0.0434035, 0.107211, -89.0372, 0.423531],                                                                                                                                                                                                 
[-0.0433355, 0.0444087, -41.1935, 0.424475],                                                                                                                                                                                                
[-0.0433025, 0.0331363, 18.6102, 0.42545],                                                                                                                                                                                                  
[-0.0432474, 0.083637, 76.9875, 0.426482],                                                                                                                                                                                                  
[-0.0431384, 0.1678, 109.787, 0.427366],                                                                                                                                                                                                    
[-0.0429178, 0.281056, 113.296, 0.42835],                                                                                                                                                                                                   
[-0.0425723, 0.377868, 63.1308, 0.429385],                                                                                                                                                                                                  
[-0.042426, 0.390902, 7.65022, 0.430379],                                                                                                                                                                                                   
[-0.0420077, 0.334971, -92.2053, 0.431503],                                                                                                                                                                                                 
[-0.0417285, 0.228925, -114.686, 0.432487],
[-0.0415687, 0.132535, -96.6896, 0.433378],                                                                                                                                                                                                 
[-0.0414755, 0.0567275, -46.3895, 0.434409],                                                                                                                                                                                                
[-0.0414325, 0.04085, 14.8824, 0.435392],                                                                                                                                                                                                   
[-0.041373, 0.0870477, 74.5041, 0.436403],                                                                                                                                                                                                  
[-0.0412299, 0.190122, 114.579, 0.437463],                                                                                                                                                                                                  
[-0.0410041, 0.299298, 115.502, 0.438386],                                                                                                                                                                                                  
[-0.0406686, 0.391942, 68.0452, 0.439346],                                                                                                                                                                                                  
[-0.0405265, 0.406033, 13.2371, 0.440355],                                                                                                                                                                                                  
[-0.0401251, 0.361029, -87.9487, 0.441378],                                                                                                                                                                                                 
[-0.0398137, 0.251743, -119.123, 0.442386],                                                                                                                                                                                                 
[-0.0396187, 0.137467, -100.234, 0.443396],                                                                                                                                                                                                 
[-0.0395209, 0.0581283, -46.6326, 0.444447],                                                                                                                                                                                                
[-0.0394789, 0.0423455, 13.1354, 0.445369],                                                                                                                                                                                                 
[-0.0394203, 0.0863824, 74.1596, 0.446357],                                                                                                                                                                                                 
[-0.0392851, 0.186078, 116.438, 0.447377],                                                                                                                                                                                                  
[-0.0390354, 0.309212, 119.646, 0.448386],                                                                                                                                                                                                  
[-0.0386742, 0.4077, 67.5662, 0.449382],                                                                                                                                                                                                    
[-0.0384884, 0.421792, -1.30717, 0.450445],                                                                                                                                                                                                 
[-0.0380949, 0.3687, -95.1809, 0.451422],                                                                                                                                                                                                   
[-0.0377851, 0.256039, -124.177, 0.452406],                                                                                                                                                                                                 
[-0.0375896, 0.138676, -103.85, 0.453406],                                                                                                                                                                                                  
[-0.0374947, 0.0588909, -50.8032, 0.454412],                                                                                                                                                                                                
[-0.0374481, 0.0421188, 19.8196, 0.455465],                                                                                                                                                                                                 
[-0.0372951, 0.164457, 113.969, 0.45717],                                                                                                                                                                                                   
[-0.0364983, 0.434836, 37.5268, 0.459702],                                                                                                                                                                                                  
[-0.0362919, 0.436117, -28.9022, 0.460471],                                                                                                                                                                                                 
[-0.0359256, 0.371141, -106.474, 0.461366],                                                                                                                                                                                                 
[-0.0356078, 0.244198, -130.94, 0.462392],                                                                                                                                                                                                  
[-0.0354224, 0.118927, -104.341, 0.463426],                                                                                                                                                                                                 
[-0.0353472, 0.0410509, -48.0169, 0.464425],                                                                                                                                                                                                
[-0.0353171, 0.0285791, 25.1549, 0.46549],                                                                                                                                                                                                  
[-0.0352643, 0.0869057, 88.0302, 0.466498],                                                                                                                                                                                                 
[-0.0351339, 0.191398, 125.31, 0.467455],                                                                                                                                                                                                   
[-0.0348843, 0.317693, 124.388, 0.468435],                                                                                                                                                                                                  
[-0.0345361, 0.414236, 71.0202, 0.469376],                                                                                                                                                                                                  
[-0.0343552, 0.429481, 3.25462, 0.470426],                                                                                                                                                                                                  
[-0.0339159, 0.369461, -99.9455, 0.471501],                                                                                                                                                                                                 
[-0.0335732, 0.236895, -124.231, 0.472623],                                                                                                                                                                                                 
[-0.0334198, 0.142519, -102.996, 0.473438],                                                                                                                                                                                                 
[-0.0333153, 0.0608269, -42.9096, 0.474525],                                                                                                                                                                                                
[-0.03327, 0.0496846, 18.5334, 0.475419],                                                                                                                                                                                                   
[-0.0332037, 0.0982868, 80.661, 0.47638],                                                                                                                                                                                                   
[-0.0330391, 0.213601, 125.909, 0.477464],                                                                                                                                                                                                  
[-0.0327601, 0.34343, 124.81, 0.478465],                                                                                                                                                                                                    
[-0.0323764, 0.441257, 66.8852, 0.479432],                                                                                                                                                                                                  
[-0.0321798, 0.454229, -4.42455, 0.480436],                                                                                                                                                                                                 
[-0.0317689, 0.398181, -101.694, 0.481384],                                                                                                                                                                                                 
[-0.0314073, 0.2641, -133.802, 0.482466],                                                                                                                                                                                                   
[-0.031214, 0.144147, -110.556, 0.483422],                                                                                                                                                                                                  
[-0.0311183, 0.0616702, -55.3079, 0.484394], 
[-0.0310737, 0.0413349, 14.4642, 0.485367],                                                                                                                                                                                                 
[-0.0310094, 0.0945222, 85.1859, 0.48641],                                                                                                                                                                                                  
[-0.0308659, 0.203102, 129.219, 0.487398],                                                                                                                                                                                                  
[-0.0305799, 0.34394, 130.538, 0.488444],                                                                                                                                                                                                   
[-0.0301821, 0.44881, 69.3409, 0.489435],                                                                                                                                                                                                   
[-0.0299829, 0.462488, -3.33812, 0.490435],                                                                                                                                                                                                 
[-0.0295704, 0.407733, -102.058, 0.491366],                
[-0.0292194, 0.281864, -136.265, 0.492376],                
[-0.0290095, 0.156664, -116.269, 0.49334],                 
[-0.0289006, 0.0642942, -57.9808, 0.494373],               
[-0.0288544, 0.0419298, 13.1712, 0.495349],                
[-0.0287931, 0.0916481, 83.4028, 0.496356],                
[-0.0286355, 0.210922, 132.471, 0.497429],                 
[-0.0283634, 0.345487, 134.021, 0.498408],                 
[-0.0279713, 0.453038, 76.2623, 0.499379],                 
[-0.0277965, 0.468868, 9.96397, 0.500378],                 
[-0.0273414, 0.414754, -102.633, 0.501386],                
[-0.026982, 0.286909, -137.636, 0.502402],                 
[-0.0267634, 0.157474, -115.879, 0.503394],                
[-0.0266564, 0.0692883, -59.0015, 0.504378],               
[-0.0266041, 0.0469496, 15.8886, 0.505389],                
[-0.0265356, 0.100378, 87.2378, 0.506402],                 
[-0.0263893, 0.209421, 132.826, 0.50737],                  
[-0.026107, 0.350252, 136.975, 0.50838],                   
[-0.0256942, 0.463752, 77.2717, 0.509382],                 
[-0.02551, 0.47979, 7.83555, 0.510389],                    
[-0.0250543, 0.424977, -104.582, 0.511376],                
[-0.024692, 0.297228, -140.931, 0.512371],                 
[-0.0244601, 0.16023, -118.821, 0.513393],                 
[-0.0243504, 0.0686204, -59.4593, 0.514396],               
[-0.024299, 0.0466059, 16.6817, 0.5154],                   
[-0.0242308, 0.101061, 89.0534, 0.516407],                 
[-0.0240859, 0.210331, 135.045, 0.51736],                  
[-0.0238023, 0.353467, 139.955, 0.518368],                 
[-0.0233601, 0.473981, 74.7391, 0.519422],                 
[-0.023156, 0.489011, -0.394429, 0.520422],                
[-0.0226779, 0.422784, -113.11, 0.521449],                 
[-0.0223087, 0.282973, -144.118, 0.522487],                
[-0.0221076, 0.158442, -120.445, 0.523406],                
[-0.0220045, 0.0698995, -64.0209, 0.524345],               
[-0.0219544, 0.0436, 12.4046, 0.525341],                   
[-0.0218877, 0.0976901, 88.9754, 0.526383],                
[-0.021738, 0.21271, 137.59, 0.527373],                    
[-0.0214487, 0.358973, 141.641, 0.528386],                 
[-0.0210369, 0.474075, 81.8311, 0.529364]])

fig, ax = plt.subplots(3,1,sharex=True)
for i in range(3):
    ax[i].plot(data[:,3], data[:,i])
    ax[i].grid()
plt.show()