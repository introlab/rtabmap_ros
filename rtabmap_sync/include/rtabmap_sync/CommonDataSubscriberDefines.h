/*
Copyright (c) 2010-2016, Mathieu Labbe - IntRoLab - Universite de Sherbrooke
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the Universite de Sherbrooke nor the
      names of its contributors may be used to endorse or promote products
      derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#ifndef INCLUDE_RTABMAP_ROS_COMMONDATASUBSCRIBERIMPL_H_
#define INCLUDE_RTABMAP_ROS_COMMONDATASUBSCRIBERIMPL_H_

#include <rtabmap/utilite/UConversion.h>
#include <rtabmap_sync/GetTopicName.h>

#define DATA_SYNC2(PREFIX, SYNC_NAME, MSG0, MSG1) \
	typedef message_filters::sync_policies::SYNC_NAME##Time<MSG0, MSG1> PREFIX##SYNC_NAME##SyncPolicy; \
	message_filters::Synchronizer<PREFIX##SYNC_NAME##SyncPolicy> * PREFIX##SYNC_NAME##Sync_;

#define DATA_SYNCS2(PREFIX, MSG0, MSG1) \
		DATA_SYNC2(PREFIX, Approximate, MSG0, MSG1) \
		DATA_SYNC2(PREFIX, Exact, MSG0, MSG1) \
		void PREFIX##Callback(const MSG0 ::ConstSharedPtr, const MSG1 ::ConstSharedPtr);

#define DATA_SYNC3(PREFIX, SYNC_NAME, MSG0, MSG1, MSG2) \
		typedef message_filters::sync_policies::SYNC_NAME##Time<MSG0, MSG1, MSG2> PREFIX##SYNC_NAME##SyncPolicy; \
		message_filters::Synchronizer<PREFIX##SYNC_NAME##SyncPolicy> * PREFIX##SYNC_NAME##Sync_;

#define DATA_SYNCS3(PREFIX, MSG0, MSG1, MSG2) \
		DATA_SYNC3(PREFIX, Approximate, MSG0, MSG1, MSG2) \
		DATA_SYNC3(PREFIX, Exact, MSG0, MSG1, MSG2) \
		void PREFIX##Callback(const MSG0 ::ConstSharedPtr, const MSG1 ::ConstSharedPtr, const MSG2 ::ConstSharedPtr);

#define DATA_SYNC4(PREFIX, SYNC_NAME, MSG0, MSG1, MSG2, MSG3) \
		typedef message_filters::sync_policies::SYNC_NAME##Time<MSG0, MSG1, MSG2, MSG3> PREFIX##SYNC_NAME##SyncPolicy; \
		message_filters::Synchronizer<PREFIX##SYNC_NAME##SyncPolicy> * PREFIX##SYNC_NAME##Sync_;

#define DATA_SYNCS4(PREFIX, MSG0, MSG1, MSG2, MSG3) \
		DATA_SYNC4(PREFIX, Approximate, MSG0, MSG1, MSG2, MSG3) \
		DATA_SYNC4(PREFIX, Exact, MSG0, MSG1, MSG2, MSG3) \
		void PREFIX##Callback(const MSG0 ::ConstSharedPtr, const MSG1 ::ConstSharedPtr, const MSG2 ::ConstSharedPtr, const MSG3 ::ConstSharedPtr);

#define DATA_SYNC5(PREFIX, SYNC_NAME, MSG0, MSG1, MSG2, MSG3, MSG4) \
		typedef message_filters::sync_policies::SYNC_NAME##Time<MSG0, MSG1, MSG2, MSG3, MSG4> PREFIX##SYNC_NAME##SyncPolicy; \
		message_filters::Synchronizer<PREFIX##SYNC_NAME##SyncPolicy> * PREFIX##SYNC_NAME##Sync_;

#define DATA_SYNCS5(PREFIX, MSG0, MSG1, MSG2, MSG3, MSG4) \
		DATA_SYNC5(PREFIX, Approximate, MSG0, MSG1, MSG2, MSG3, MSG4) \
		DATA_SYNC5(PREFIX, Exact, MSG0, MSG1, MSG2, MSG3, MSG4) \
		void PREFIX##Callback(const MSG0 ::ConstSharedPtr, const MSG1 ::ConstSharedPtr, const MSG2 ::ConstSharedPtr, const MSG3 ::ConstSharedPtr, const MSG4 ::ConstSharedPtr);

#define DATA_SYNC6(PREFIX, SYNC_NAME, MSG0, MSG1, MSG2, MSG3, MSG4, MSG5) \
		typedef message_filters::sync_policies::SYNC_NAME##Time<MSG0, MSG1, MSG2, MSG3, MSG4, MSG5> PREFIX##SYNC_NAME##SyncPolicy; \
		message_filters::Synchronizer<PREFIX##SYNC_NAME##SyncPolicy> * PREFIX##SYNC_NAME##Sync_;

#define DATA_SYNCS6(PREFIX, MSG0, MSG1, MSG2, MSG3, MSG4, MSG5) \
		DATA_SYNC6(PREFIX, Approximate, MSG0, MSG1, MSG2, MSG3, MSG4, MSG5) \
		DATA_SYNC6(PREFIX, Exact, MSG0, MSG1, MSG2, MSG3, MSG4, MSG5) \
		void PREFIX##Callback(const MSG0 ::ConstSharedPtr, const MSG1 ::ConstSharedPtr, const MSG2 ::ConstSharedPtr, const MSG3 ::ConstSharedPtr, const MSG4 ::ConstSharedPtr, const MSG5 ::ConstSharedPtr);

#define DATA_SYNC7(PREFIX, SYNC_NAME, MSG0, MSG1, MSG2, MSG3, MSG4, MSG5, MSG6) \
		typedef message_filters::sync_policies::SYNC_NAME##Time<MSG0, MSG1, MSG2, MSG3, MSG4, MSG5, MSG6> PREFIX##SYNC_NAME##SyncPolicy; \
		message_filters::Synchronizer<PREFIX##SYNC_NAME##SyncPolicy> * PREFIX##SYNC_NAME##Sync_;

#define DATA_SYNCS7(PREFIX, MSG0, MSG1, MSG2, MSG3, MSG4, MSG5, MSG6) \
		DATA_SYNC7(PREFIX, Approximate, MSG0, MSG1, MSG2, MSG3, MSG4, MSG5, MSG6) \
		DATA_SYNC7(PREFIX, Exact, MSG0, MSG1, MSG2, MSG3, MSG4, MSG5, MSG6) \
		void PREFIX##Callback(const MSG0 ::ConstSharedPtr, const MSG1 ::ConstSharedPtr, const MSG2 ::ConstSharedPtr, const MSG3 ::ConstSharedPtr, const MSG4 ::ConstSharedPtr, const MSG5 ::ConstSharedPtr, const MSG6 ::ConstSharedPtr);

#define DATA_SYNC8(PREFIX, SYNC_NAME, MSG0, MSG1, MSG2, MSG3, MSG4, MSG5, MSG6, MSG7) \
		typedef message_filters::sync_policies::SYNC_NAME##Time<MSG0, MSG1, MSG2, MSG3, MSG4, MSG5, MSG6, MSG7> PREFIX##SYNC_NAME##SyncPolicy; \
		message_filters::Synchronizer<PREFIX##SYNC_NAME##SyncPolicy> * PREFIX##SYNC_NAME##Sync_;

#define DATA_SYNCS8(PREFIX, MSG0, MSG1, MSG2, MSG3, MSG4, MSG5, MSG6, MSG7) \
		DATA_SYNC8(PREFIX, Approximate, MSG0, MSG1, MSG2, MSG3, MSG4, MSG5, MSG6, MSG7) \
		DATA_SYNC8(PREFIX, Exact, MSG0, MSG1, MSG2, MSG3, MSG4, MSG5, MSG6, MSG7) \
		void PREFIX##Callback(const MSG0 ::ConstSharedPtr, const MSG1 ::ConstSharedPtr, const MSG2 ::ConstSharedPtr, const MSG3 ::ConstSharedPtr, const MSG4 ::ConstSharedPtr, const MSG5 ::ConstSharedPtr, const MSG6 ::ConstSharedPtr, const MSG7 ::ConstSharedPtr);


// Constructor
#define SYNC_INIT(PREFIX) \
	PREFIX##ApproximateSync_(0), \
	PREFIX##ExactSync_(0)

// Destructor
#define SYNC_DEL(PREFIX) \
	if(PREFIX##ApproximateSync_) delete PREFIX##ApproximateSync_; \
	if(PREFIX##ExactSync_) delete PREFIX##ExactSync_;

// Sync declarations
#define SYNC_DECL2(CLASS, PREFIX, APPROX, QUEUE_SIZE, SUB0, SUB1) \
		if(APPROX) \
		{ \
			PREFIX##ApproximateSync_ = new message_filters::Synchronizer<PREFIX##ApproximateSyncPolicy>( \
					PREFIX##ApproximateSyncPolicy(QUEUE_SIZE), SUB0, SUB1); \
			PREFIX##ApproximateSync_->registerCallback(std::bind(&CLASS::PREFIX##Callback, this, std::placeholders::_1, std::placeholders::_2)); \
		} \
		else \
		{ \
			PREFIX##ExactSync_ = new message_filters::Synchronizer<PREFIX##ExactSyncPolicy>( \
					PREFIX##ExactSyncPolicy(QUEUE_SIZE), SUB0, SUB1); \
			PREFIX##ExactSync_->registerCallback(std::bind(&CLASS::PREFIX##Callback, this, std::placeholders::_1, std::placeholders::_2)); \
		} \
		subscribedTopicsMsg_ = uFormat("\n%s subscribed to (%s sync):\n   %s \\\n   %s", \
				name_.c_str(), \
				APPROX?"approx":"exact", \
				getTopicName(SUB0.getSubscriber()).c_str(), \
				getTopicName(SUB1.getSubscriber()).c_str());

#define SYNC_DECL3(CLASS, PREFIX, APPROX, QUEUE_SIZE, SUB0, SUB1, SUB2) \
		if(APPROX) \
		{ \
			PREFIX##ApproximateSync_ = new message_filters::Synchronizer<PREFIX##ApproximateSyncPolicy>( \
					PREFIX##ApproximateSyncPolicy(QUEUE_SIZE), SUB0, SUB1, SUB2); \
			PREFIX##ApproximateSync_->registerCallback(std::bind(&CLASS::PREFIX##Callback, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3)); \
		} \
		else \
		{ \
			PREFIX##ExactSync_ = new message_filters::Synchronizer<PREFIX##ExactSyncPolicy>( \
					PREFIX##ExactSyncPolicy(QUEUE_SIZE), SUB0, SUB1, SUB2); \
			PREFIX##ExactSync_->registerCallback(std::bind(&CLASS::PREFIX##Callback, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3)); \
		} \
		subscribedTopicsMsg_ = uFormat("\n%s subscribed to (%s sync):\n   %s \\\n   %s \\\n   %s", \
				name_.c_str(), \
				APPROX?"approx":"exact", \
				getTopicName(SUB0.getSubscriber()).c_str(), \
				getTopicName(SUB1.getSubscriber()).c_str(), \
				getTopicName(SUB2.getSubscriber()).c_str());

#define SYNC_DECL4(CLASS, PREFIX, APPROX, QUEUE_SIZE, SUB0, SUB1, SUB2, SUB3) \
		if(APPROX) \
		{ \
			PREFIX##ApproximateSync_ = new message_filters::Synchronizer<PREFIX##ApproximateSyncPolicy>( \
					PREFIX##ApproximateSyncPolicy(QUEUE_SIZE), SUB0, SUB1, SUB2, SUB3); \
			PREFIX##ApproximateSync_->registerCallback(std::bind(&CLASS::PREFIX##Callback, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4)); \
		} \
		else \
		{ \
			PREFIX##ExactSync_ = new message_filters::Synchronizer<PREFIX##ExactSyncPolicy>( \
					PREFIX##ExactSyncPolicy(QUEUE_SIZE), SUB0, SUB1, SUB2, SUB3); \
			PREFIX##ExactSync_->registerCallback(std::bind(&CLASS::PREFIX##Callback, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4)); \
		} \
		subscribedTopicsMsg_ = uFormat("\n%s subscribed to (%s sync):\n   %s \\\n   %s \\\n   %s \\\n   %s", \
				name_.c_str(), \
				APPROX?"approx":"exact", \
				getTopicName(SUB0.getSubscriber()).c_str(), \
				getTopicName(SUB1.getSubscriber()).c_str(), \
				getTopicName(SUB2.getSubscriber()).c_str(), \
				getTopicName(SUB3.getSubscriber()).c_str());

#define SYNC_DECL5(CLASS, PREFIX, APPROX, QUEUE_SIZE, SUB0, SUB1, SUB2, SUB3, SUB4) \
		if(APPROX) \
		{ \
			PREFIX##ApproximateSync_ = new message_filters::Synchronizer<PREFIX##ApproximateSyncPolicy>( \
					PREFIX##ApproximateSyncPolicy(QUEUE_SIZE), SUB0, SUB1, SUB2, SUB3, SUB4); \
			PREFIX##ApproximateSync_->registerCallback(std::bind(&CLASS::PREFIX##Callback, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4, std::placeholders::_5)); \
		} \
		else \
		{ \
			PREFIX##ExactSync_ = new message_filters::Synchronizer<PREFIX##ExactSyncPolicy>( \
					PREFIX##ExactSyncPolicy(QUEUE_SIZE), SUB0, SUB1, SUB2, SUB3, SUB4); \
			PREFIX##ExactSync_->registerCallback(std::bind(&CLASS::PREFIX##Callback, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4, std::placeholders::_5)); \
		} \
		subscribedTopicsMsg_ = uFormat("\n%s subscribed to (%s sync):\n   %s \\\n   %s \\\n   %s \\\n   %s \\\n   %s", \
				name_.c_str(), \
				APPROX?"approx":"exact", \
				getTopicName(SUB0.getSubscriber()).c_str(), \
				getTopicName(SUB1.getSubscriber()).c_str(), \
				getTopicName(SUB2.getSubscriber()).c_str(), \
				getTopicName(SUB3.getSubscriber()).c_str(), \
				getTopicName(SUB4.getSubscriber()).c_str());

#define SYNC_DECL6(CLASS, PREFIX, APPROX, QUEUE_SIZE, SUB0, SUB1, SUB2, SUB3, SUB4, SUB5) \
		if(APPROX) \
		{ \
			PREFIX##ApproximateSync_ = new message_filters::Synchronizer<PREFIX##ApproximateSyncPolicy>( \
					PREFIX##ApproximateSyncPolicy(QUEUE_SIZE), SUB0, SUB1, SUB2, SUB3, SUB4, SUB5); \
			PREFIX##ApproximateSync_->registerCallback(std::bind(&CLASS::PREFIX##Callback, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4, std::placeholders::_5, std::placeholders::_6)); \
		} \
		else \
		{ \
			PREFIX##ExactSync_ = new message_filters::Synchronizer<PREFIX##ExactSyncPolicy>( \
					PREFIX##ExactSyncPolicy(QUEUE_SIZE), SUB0, SUB1, SUB2, SUB3, SUB4, SUB5); \
			PREFIX##ExactSync_->registerCallback(std::bind(&CLASS::PREFIX##Callback, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4, std::placeholders::_5, std::placeholders::_6)); \
		} \
		subscribedTopicsMsg_ = uFormat("\n%s subscribed to (%s sync):\n   %s \\\n   %s \\\n   %s \\\n   %s \\\n   %s \\\n   %s", \
				name_.c_str(), \
				APPROX?"approx":"exact", \
				getTopicName(SUB0.getSubscriber()).c_str(), \
				getTopicName(SUB1.getSubscriber()).c_str(), \
				getTopicName(SUB2.getSubscriber()).c_str(), \
				getTopicName(SUB3.getSubscriber()).c_str(), \
				getTopicName(SUB4.getSubscriber()).c_str(), \
				getTopicName(SUB5.getSubscriber()).c_str());

#define SYNC_DECL7(CLASS, PREFIX, APPROX, QUEUE_SIZE, SUB0, SUB1, SUB2, SUB3, SUB4, SUB5, SUB6) \
		if(APPROX) \
		{ \
			PREFIX##ApproximateSync_ = new message_filters::Synchronizer<PREFIX##ApproximateSyncPolicy>( \
					PREFIX##ApproximateSyncPolicy(QUEUE_SIZE), SUB0, SUB1, SUB2, SUB3, SUB4, SUB5, SUB6); \
			PREFIX##ApproximateSync_->registerCallback(std::bind(&CLASS::PREFIX##Callback, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4, std::placeholders::_5, std::placeholders::_6, std::placeholders::_7)); \
		} \
		else \
		{ \
			PREFIX##ExactSync_ = new message_filters::Synchronizer<PREFIX##ExactSyncPolicy>( \
					PREFIX##ExactSyncPolicy(QUEUE_SIZE), SUB0, SUB1, SUB2, SUB3, SUB4, SUB5, SUB6); \
			PREFIX##ExactSync_->registerCallback(std::bind(&CLASS::PREFIX##Callback, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4, std::placeholders::_5, std::placeholders::_6, std::placeholders::_7)); \
		} \
		subscribedTopicsMsg_ = uFormat("\n%s subscribed to (%s sync):\n   %s \\\n   %s \\\n   %s \\\n   %s \\\n   %s \\\n   %s \\\n   %s", \
				name_.c_str(), \
				APPROX?"approx":"exact", \
				getTopicName(SUB0.getSubscriber()).c_str(), \
				getTopicName(SUB1.getSubscriber()).c_str(), \
				getTopicName(SUB2.getSubscriber()).c_str(), \
				getTopicName(SUB3.getSubscriber()).c_str(), \
				getTopicName(SUB4.getSubscriber()).c_str(), \
				getTopicName(SUB5.getSubscriber()).c_str(), \
				getTopicName(SUB6.getSubscriber()).c_str());

#define SYNC_DECL8(CLASS, PREFIX, APPROX, QUEUE_SIZE, SUB0, SUB1, SUB2, SUB3, SUB4, SUB5, SUB6, SUB7) \
		if(APPROX) \
		{ \
			PREFIX##ApproximateSync_ = new message_filters::Synchronizer<PREFIX##ApproximateSyncPolicy>( \
					PREFIX##ApproximateSyncPolicy(QUEUE_SIZE), SUB0, SUB1, SUB2, SUB3, SUB4, SUB5, SUB6, SUB7); \
			PREFIX##ApproximateSync_->registerCallback(std::bind(&CLASS::PREFIX##Callback, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4, std::placeholders::_5, std::placeholders::_6, std::placeholders::_7, std::placeholders::_8)); \
		} \
		else \
		{ \
			PREFIX##ExactSync_ = new message_filters::Synchronizer<PREFIX##ExactSyncPolicy>( \
					PREFIX##ExactSyncPolicy(QUEUE_SIZE), SUB0, SUB1, SUB2, SUB3, SUB4, SUB5, SUB6, SUB7); \
			PREFIX##ExactSync_->registerCallback(std::bind(&CLASS::PREFIX##Callback, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4, std::placeholders::_5, std::placeholders::_6, std::placeholders::_7, std::placeholders::_8)); \
		} \
		subscribedTopicsMsg_ = uFormat("\n%s subscribed to (%s sync):\n   %s \\\n   %s \\\n   %s \\\n   %s \\\n   %s \\\n   %s \\\n   %s \\\n   %s", \
				name_.c_str(), \
				APPROX?"approx":"exact", \
				getTopicName(SUB0.getSubscriber()).c_str(), \
				getTopicName(SUB1.getSubscriber()).c_str(), \
				getTopicName(SUB2.getSubscriber()).c_str(), \
				getTopicName(SUB3.getSubscriber()).c_str(), \
				getTopicName(SUB4.getSubscriber()).c_str(), \
				getTopicName(SUB5.getSubscriber()).c_str(), \
				getTopicName(SUB6.getSubscriber()).c_str(), \
				getTopicName(SUB7.getSubscriber()).c_str());


#endif /* INCLUDE_RTABMAP_ROS_COMMONDATASUBSCRIBERIMPL_H_ */
