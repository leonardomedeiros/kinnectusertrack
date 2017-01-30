using System;

using Microsoft.Kinect;
using System.Numerics;

using XnaGeometry;


namespace KinectUserHeight
{
    /// <summary>
    /// Provides some common functionality on skeletal data.
    /// </summary>
    public static class SkeletalExtensions
    {
        #region Public methods

        const double CONSTANT_RADIANUS_TO_GRAUS = 57.3;
        
        /// <summary>
        /// Returns the height of the specified skeleton.
        /// </summary>
        /// <param name="skeleton">The specified user skeleton.</param>
        /// <returns>The height of the skeleton in meters.</returns>
        public static double Height(this Skeleton skeleton)
        {
            const double HEAD_DIVERGENCE = 0.1;

            var head = skeleton.Joints[JointType.Head];
            var neck = skeleton.Joints[JointType.ShoulderCenter];
            var spine = skeleton.Joints[JointType.Spine];
            var waist = skeleton.Joints[JointType.HipCenter];
            var hipLeft = skeleton.Joints[JointType.HipLeft];
            var hipRight = skeleton.Joints[JointType.HipRight];
            var kneeLeft = skeleton.Joints[JointType.KneeLeft];
            var kneeRight = skeleton.Joints[JointType.KneeRight];
            var ankleLeft = skeleton.Joints[JointType.AnkleLeft];
            var ankleRight = skeleton.Joints[JointType.AnkleRight];
            var footLeft = skeleton.Joints[JointType.FootLeft];
            var footRight = skeleton.Joints[JointType.FootRight];

            // Find which leg is tracked more accurately.
            int legLeftTrackedJoints = NumberOfTrackedJoints(hipLeft, kneeLeft, ankleLeft, footLeft);
            int legRightTrackedJoints = NumberOfTrackedJoints(hipRight, kneeRight, ankleRight, footRight);

            double legLength = legLeftTrackedJoints > legRightTrackedJoints ? Length(hipLeft, kneeLeft, ankleLeft, footLeft) : Length(hipRight, kneeRight, ankleRight, footRight);

            return Length(head, neck, spine, waist) + legLength + HEAD_DIVERGENCE;
        }

        /// <summary>
        /// Returns the left hand position of the specified skeleton.
        /// Acording Joint Hierarchy
        /// </summary>
        /// <param name="skeleton">The specified user skeleton.</param>
        /// <returns>The position of the left hand.</returns>
        public static double LeftHand(this Skeleton skeleton)
        {
            var shoulderLeft = skeleton.Joints[JointType.ShoulderLeft];
            var elbowLeft = skeleton.Joints[JointType.ElbowLeft];
            var wristLeft = skeleton.Joints[JointType.WristLeft];
            var handLeft = skeleton.Joints[JointType.HandLeft];

            //return Length(shoulderLeft, elbowLeft, wristLeft, handLeft);
            //returnL ength(hipLeft, handLeft);
            return Length(shoulderLeft,elbowLeft,wristLeft,handLeft);
        }

        /// <summary>
        /// O Ângulo relativo do braço é calculado usando a Lei dos Cossenos. Essa lei é simplesmmente um caso mais geral do Teorema 
        /// de Pitágoras e descreve a relação entre os lados de um triângulo. Os ângulos relativos podem ser calculados usando 
        /// e Lei dos Cossenos. Essa lei é simplesmente um caso mais geral do Teorema de Pitágoras e descreve a relação entre os 
        /// lados de um triângulo. Para nossos propósitos, o triângulo é constituído por dois segmentos (b e c) e uma linha (a)
        /// unindo a ponta distai de um segmento com a ponta proximal do outro.
        /// Então iremos calcular a distância Euclidiana do Ombro Esquerdo até o Cotovelo como Sendo (B). A distância do cotovelo 
        /// até o punho como sendo C. E a distância euclidiana do punho até o Ombro como sendo A.
        /// </summary>
        /// <param name="skeleton">The specified user skeleton.</param>
        /// <returns>The position of the left hand.</returns>
        public static double LeftArmRelativeAngle(this Skeleton skeleton)
        {
            //
            var shoulderLeft = skeleton.Joints[JointType.ShoulderLeft];
            var elbowLeft = skeleton.Joints[JointType.ElbowLeft];
            var wristLeft = skeleton.Joints[JointType.WristLeft];

            double B = EuclidianDistance(shoulderLeft, elbowLeft);
            double C = EuclidianDistance(elbowLeft, wristLeft);
            double A = EuclidianDistance(shoulderLeft, wristLeft);

            //A formula da lei dos cosenos é:
            //a^2 = b^2 + c^2 - 2*b*c*cos(Teta)
            //que resulta em:
            //cos(teta) = (b^2 + c^2 - a^2)/2*b*c;
            double cosTeta = (Math.Pow(B,2) + Math.Pow(C,2) - Math.Pow(A,2)) / 2 * B * C;

            //para encontrar o ângulo em radianos calculamos a inversa do coseno que teremos
            double radianos = Math.Acos(cosTeta);

            //para converter o ângulo em graus deve-se multiplicar pela constante 57,3;
            double graus = radianos * CONSTANT_RADIANUS_TO_GRAUS;


            //return Length(shoulderLeft, elbowLeft, wristLeft, handLeft);
            //returnL ength(hipLeft, handLeft);
            //return graus;
            return 6;
        }

        /// <summary>
        /// 
        /// </summary>
        /* MATLAB - CODE
         * shoulder = [torsoJoint(Index,2),torsoJoint(Index,3), torsoJoint(Index,4)];
           wrist = [shoulderJoint(Index,2),shoulderJoint(Index,3), shoulderJoint(Index,4)];
           hip = [elbowJoint(Index,2),elbowJoint(Index,3), elbowJoint(Index,4)];
           u = wrist-shoulder;
           v = hip-shoulder;

           CosTheta = dot(u,v)/(norm(u)*norm(v));
           ThetaInDegrees = acos(CosTheta)*180/pi;
           angle = ThetaInDegrees;
        */
        public static double RightArmRelativeAngle(this Skeleton skeleton)
        {
            var shoulder = skeleton.Joints[JointType.ShoulderRight];
            var wrist = skeleton.Joints[JointType.WristRight];
            var hip = skeleton.Joints[JointType.HipRight];
            //Vector3 v = new Vector3(3,3,3);
            Vector3 u = new Vector3(wrist.Position.X - shoulder.Position.X, wrist.Position.Y - shoulder.Position.Y,
                wrist.Position.Z - shoulder.Position.Z );
            Vector3 v = new Vector3(hip.Position.X - shoulder.Position.X, hip.Position.Y - shoulder.Position.Y,
                hip.Position.Z - shoulder.Position.Z);

            Double cosTheta = Vector3.Dot(Vector3.Normalize(u), Vector3.Normalize(v));
           
            Double ThetaInDegrees = Math.Acos(cosTheta) * 180 / Math.PI;


            return ThetaInDegrees;            
        }

        public static double NeckRelativeAngle(this Skeleton skeleton)
        {
            var shoulderRight = skeleton.Joints[JointType.ShoulderRight];
            var head = skeleton.Joints[JointType.Head];
            var elbowRight = skeleton.Joints[JointType.ElbowRight];
            //Vector3 v = new Vector3(3,3,3);
            Vector3 u = new Vector3(head.Position.X - shoulderRight.Position.X, head.Position.Y - shoulderRight.Position.Y,
                head.Position.Z - shoulderRight.Position.Z);
            Vector3 v = new Vector3(elbowRight.Position.X - shoulderRight.Position.X, elbowRight.Position.Y - shoulderRight.Position.Y,
                elbowRight.Position.Z - shoulderRight.Position.Z);

            Double cosTheta = Vector3.Dot(Vector3.Normalize(u), Vector3.Normalize(v));

            Double ThetaInDegrees = Math.Acos(cosTheta) * 180 / Math.PI;


            return ThetaInDegrees;
        }


        public static double RightHand(this Skeleton skeleton)
        {
            var shoulderRight = skeleton.Joints[JointType.ShoulderRight];
            //var hipCenter = skeleton.Joints[JointType.HipCenter];
            //var elbowLeft = skeleton.Joints[JointType.ElbowLeft];
            //var wristLeft = skeleton.Joints[JointType.WristLeft];
            var handRight = skeleton.Joints[JointType.HandRight];

            //return Length(shoulderLeft, elbowLeft, wristLeft, handLeft);
            //returnL ength(hipLeft, handLeft);
            return Length(shoulderRight, handRight);
        }

        /// <summary>
        /// Returns the upper height of the specified skeleton (head to waist). Useful whenever Kinect provides a way to track seated users.
        /// </summary>
        /// <param name="skeleton">The specified user skeleton.</param>
        /// <returns>The upper height of the skeleton in meters.</returns>
        public static double UpperHeight(this Skeleton skeleton)
        {
            var head = skeleton.Joints[JointType.Head];
            var neck = skeleton.Joints[JointType.ShoulderCenter];
            var spine = skeleton.Joints[JointType.Spine];
            var waist = skeleton.Joints[JointType.HipCenter];

            return Length(head, neck, spine, waist);
        }

        /// <summary>
        /// Returns the length of the segment defined by the specified joints.
        /// </summary>
        /// <param name="p1">The first joint (start of the segment).</param>
        /// <param name="p2">The second joint (end of the segment).</param>
        /// <returns>The length of the segment in meters.</returns>
        public static double Length(Joint p1, Joint p2)
        {
            return Math.Sqrt(
                Math.Pow(p1.Position.X - p2.Position.X, 2) +
                Math.Pow(p1.Position.Y - p2.Position.Y, 2) +
                Math.Pow(p1.Position.Z - p2.Position.Z, 2));
        }

        /// <summary>
        /// Returns the Euclidian Distance of the segment defined by the specified joints.
        /// </summary>
        /// <param name="p1">The first joint (start of the segment).</param>
        /// <param name="p2">The second joint (end of the segment).</param>
        /// <returns>The length of the segment in meters.</returns>
        public static double EuclidianDistance(Joint p1, Joint p2)
        {
            return Math.Sqrt(
                Math.Pow(p1.Position.X - p2.Position.X, 2) +
                Math.Pow(p1.Position.Y - p2.Position.Y, 2));
        }

        /// <summary>
        /// Returns the length of the segments defined by the specified joints.
        /// </summary>
        /// <param name="joints">A collection of two or more joints.</param>
        /// <returns>The length of all the segments in meters.</returns>
        public static double Length(params Joint[] joints)
        {
            double length = 0;

            for (int index = 0; index < joints.Length - 1; index++)
            {
                length += Length(joints[index], joints[index + 1]);
            }

            return length;
        }

        /// <summary>
        /// Given a collection of joints, calculates the number of the joints that are tracked accurately.
        /// </summary>
        /// <param name="joints">A collection of joints.</param>
        /// <returns>The number of the accurately tracked joints.</returns>
        public static int NumberOfTrackedJoints(params Joint[] joints)
        {
            int trackedJoints = 0;

            foreach (var joint in joints)
            {
                if (joint.TrackingState == JointTrackingState.Tracked)
                {
                    trackedJoints++;
                }
            }

            return trackedJoints;
        }

        /// <summary>
        /// Scales the specified joint according to the specified dimensions.
        /// </summary>
        /// <param name="joint">The joint to scale.</param>
        /// <param name="width">Width.</param>
        /// <param name="height">Height.</param>
        /// <param name="skeletonMaxX">Maximum X.</param>
        /// <param name="skeletonMaxY">Maximum Y.</param>
        /// <returns>The scaled version of the joint.</returns>
        public static Joint ScaleTo(this Joint joint, int width, int height, float skeletonMaxX, float skeletonMaxY)
        {
            SkeletonPoint position = new SkeletonPoint()
            {
                X = Scale(width, skeletonMaxX, joint.Position.X),
                Y = Scale(height, skeletonMaxY, -joint.Position.Y),
                Z = joint.Position.Z
            };

            joint.Position = position;

            return joint;
        }

        /// <summary>
        /// Scales the specified joint according to the specified dimensions.
        /// </summary>
        /// <param name="joint">The joint to scale.</param>
        /// <param name="width">Width.</param>
        /// <param name="height">Height.</param>
        /// <returns>The scaled version of the joint.</returns>
        public static Joint ScaleTo(this Joint joint, int width, int height)
        {
            return ScaleTo(joint, width, height, 1.0f, 1.0f);
        }

        #endregion

        #region Helpers

        /// <summary>
        /// Returns the scaled value of the specified position.
        /// </summary>
        /// <param name="maxPixel">Width or height.</param>
        /// <param name="maxSkeleton">Border (X or Y).</param>
        /// <param name="position">Original position (X or Y).</param>
        /// <returns>The scaled value of the specified position.</returns>
        private static float Scale(int maxPixel, float maxSkeleton, float position)
        {
            float value = ((((maxPixel / maxSkeleton) / 2) * position) + (maxPixel / 2));

            if (value > maxPixel)
            {
                return maxPixel;
            }

            if (value < 0)
            {
                return 0;
            }

            return value;
        }

        #endregion
    }
}
