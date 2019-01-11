/*
 * Copyright (C) 2018-2019 LEIDOS.
 *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may not
 * use this file except in compliance with the License. You may obtain a copy of
 * the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
 * License for the specific language governing permissions and limitations under
 * the License.
 */

package gov.dot.fhwa.saxton.carma.guidance.pubsub;

/**
 * Generic means to instantiate new {@link IPublicationChannel} instances for new topics
 */
public interface IPublicationChannelFactory {
    /**
     * Create a new {@link IPublicationChannel} for the specified topic, type, and message type
     * parameter
     *
     * @param topic The URL for the topic to be published to
     * @param type  The String identifier of the message type
     * @param <T>   Type parameter for the message class associated with the type value
     * @return A new instance of IPublicationChannel holding all resources needed for publication
     */
    <T> IPublicationChannel<T> newPublicationChannel(String topic, String type);
}
