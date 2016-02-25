package dragonBones
{
	import flash.geom.Point;
	
	import dragonBones.objects.IKData;
	
	public class IKConstraint
	{
		private var ikdata:IKData;
		private var armature:Armature;
		
		public var bones:Vector.<Bone>;
		public var target:Bone;
		public var bendDirection:int;
		public var weight:Number;
		
		public var animationCacheBend:int=0;		
		public var animationCacheWeight:Number=-1;	
		
		public function IKConstraint(data:IKData,armatureData:Armature)
		{
			this.ikdata = data;
			this.armature = armatureData
				
			weight = data.weight;
			bendDirection = (data.bendPositive?1:-1);
			bones = new Vector.<Bone>();
			var bone:Bone;
			if(data.chain){
				bone = armatureData.getBone(data.bones).parent;
				bone.isIKConstraint = true;
				bones.push(bone);
			}
			bone = armatureData.getBone(data.bones);
			bone.isIKConstraint = true;
			bones.push(bone);
			target = armatureData.getBone(data.target);
		}
		public function dispose():void
		{
			
		}
		public function compute():void
		{
			switch (bones.length) {
				case 1:
					var weig1:Number = animationCacheWeight>=0?animationCacheWeight:weight;
					compute1(bones[0], target, weig1);
					break;
				case 2:
					var bend:int = animationCacheBend!=0?animationCacheBend:bendDirection;
					var weig:Number = animationCacheWeight>=0?animationCacheWeight:weight;
					var tt:Point = compute2(bones[0],bones[1],target.global.x,target.global.y, bend, weig);
					bones[0].rotationIK = tt.x;
					bones[1].rotationIK = tt.y+tt.x;
					bones[0].ikDvalue = bones[0].rotationIK-bones[0].global.rotation;
					bones[1].ikDvalue = bones[1].rotationIK-bones[1].global.rotation;
					break;
			}
		}
		public function compute1 (bone:Bone, target:Bone, weightA:Number) : void {
			var parentRotation:Number = (!bone.inheritRotation || bone.parent == null) ? 0 : bone.parent.global.rotation;
			var rotation:Number = bone.global.rotation;
			var rotationIK:Number = Math.atan2(target.global.y - bone.global.y, target.global.x - bone.global.x);
			bone.rotationIK = rotation + (rotationIK - rotation) * weightA;
			bone.ikDvalue = bone.rotationIK-rotation;
		}
		public function compute2(parent:Bone, child:Bone, targetX:Number,targetY:Number, bendDirection:int, weightA:Number):Point
		{
			//添加斜切后的算法，现在用的
			if (weightA == 0) {
				return new Point(parent.global.rotation,child.global.rotation);
			}
			var tt:Point = new Point();
			/**父的绝对坐标**/
			var p1:Point = new Point(parent.global.x,parent.global.y);
			/**子的绝对坐标**/
			var p2:Point = new Point(child.global.x,child.global.y);
			var psx:Number = parent.global.scaleX;
			var psy:Number = parent.global.scaleY;
			var csx:Number = child.global.scaleX;
			var csy:Number = child.global.scaleY;
			
			var cx:Number = child.origin.x*psx;
			var cy:Number = child.origin.y*psy;
			var initalRotation:Number = Math.atan2(cy, cx);//差值等于子在父落点到父的角度
			
			var childX:Number = p2.x-p1.x;
			var childY:Number = p2.y-p1.y;
			/**d1的长度**/
			var len1:Number = Math.sqrt(childX * childX + childY* childY);
			var parentAngle:Number;
			var childAngle:Number;
			outer:
			if (Math.abs(psx - psy) <= 0.001) {
				var childlength:Number = child.length;
				var len2:Number = childlength*csx;
				targetX = targetX-p1.x;
				targetY = targetY-p1.y;
				var cosDenom:Number = 2 * len1 * len2;
				if (cosDenom < 0.0001) {
					var temp:Number = Math.atan2(targetY, targetX);
					tt.x = temp  * weightA - initalRotation;
					tt.y = temp  * weightA + initalRotation//+ tt.x ;
					normalize(tt.x);
					normalize(tt.y);
					return tt;
				}
				var cos:Number = (targetX * targetX + targetY * targetY - len1 * len1 - len2 * len2) / cosDenom;
				if (cos < -1)
					cos = -1;
				else if (cos > 1)
					cos = 1;
				childAngle = Math.acos(cos) * bendDirection;//o2
				var adjacent:Number = len1 + len2 * cos;  //ae
				var opposite:Number = len2 * Math.sin(childAngle);//be
				parentAngle = Math.atan2(targetY * adjacent - targetX * opposite, targetX * adjacent + targetY * opposite);//o1
				tt.x = parentAngle * weightA-initalRotation;
				tt.y = childAngle* weightA+initalRotation;//+tt.x;
			}else{//一旦父已经扭曲，子重新计算长度
				var l1:Number = len1;
				var tx:Number = targetX-p1.x;
				var ty:Number = targetY-p1.y;
				var l2:Number = child.length*child.origin.scaleX;//child.currentLocalTransform.scaleX;
				var a:Number = psx * l2;
				var b:Number = psy * l2;
				var ta:Number = Math.atan2(ty, tx);
				var aa:Number = a * a;
				var bb:Number = b * b;
				var ll:Number = l1 * l1;
				var dd:Number = tx * tx + ty * ty;
				var c0:Number = bb * ll + aa * dd - aa * bb;
				var c1:Number = -2 * bb * l1;
				var c2:Number = bb - aa;
				var d:Number = c1 * c1 - 4 * c2 * c0;
				if (d >= 0) {
					var q:Number =Math.sqrt(d);
					if (c1 < 0) q = -q;
					q = -(c1 + q) / 2;
					var r0:Number = q / c2
					var r1:Number = c0 / q;
					var r:Number = Math.abs(r0) < Math.abs(r1) ? r0 : r1;
					if (r * r <= dd) {
						var y1:Number = Math.sqrt(dd - r * r) * bendDirection;
						parentAngle = ta - Math.atan2(y1, r);
						childAngle = Math.atan2(y1 / psy, (r - l1) / psx);
						tt.x = parentAngle* weightA-initalRotation;
						tt.y = childAngle* weightA+initalRotation;//+tt.x;
						break outer;
					}
				}
				var minAngle:Number = 0;
				var minDist:Number = Number.MAX_VALUE;
				var minX:Number = 0;
				var minY:Number = 0;
				var maxAngle:Number = 0;
				var maxDist:Number = 0;
				var maxX:Number = 0;
				var maxY:Number = 0;
				var x2:Number = l1 + a;
				var dist:Number = x2 * x2;
				if (dist > maxDist) {
					maxAngle = 0;
					maxDist = dist;
					maxX = x2;
				}
				x2 = l1 - a;
				dist = x2 * x2;
				if (dist < minDist) {
					minAngle = Math.PI;
					minDist = dist;
					minX = x2;
				}
				var angle1:Number = Math.acos(-a * l1 / (aa - bb));
				x2 = a * Math.cos(angle1) + l1;
				var y2:Number = b * Math.sin(angle1);
				dist = x2 * x2 + y2 * y2;
				if (dist < minDist) {
					minAngle = angle1;
					minDist = dist;
					minX = x2;
					minY = y2;
				}
				if (dist > maxDist) {
					maxAngle = angle1;
					maxDist = dist;
					maxX = x2;
					maxY = y2;
				}
				if (dd <= (minDist + maxDist) / 2) {
					parentAngle = ta - Math.atan2(minY * bendDirection, minX);
					childAngle = minAngle * bendDirection;
				} else {
					parentAngle = ta - Math.atan2(maxY * bendDirection, maxX);
					childAngle = maxAngle * bendDirection;
				}
				tt.x = parentAngle* weightA-initalRotation;
				tt.y = childAngle* weightA+initalRotation;//;
			}
			normalize(tt.x);
			normalize(tt.y);
			return tt;
		}
		private function normalize(rotation:Number):void
		{
			if (rotation > Math.PI)
				rotation -= Math.PI*2;
			else if (rotation < -Math.PI)
				rotation += Math.PI*2;
		}
	}
}